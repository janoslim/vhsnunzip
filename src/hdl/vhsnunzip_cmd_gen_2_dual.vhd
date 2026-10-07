library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Speculative dual-issue command generator stage 2.
--
-- Slot 0 is the single-issue address generator (vhsnunzip_cmd_gen_2): it splits
-- long literals over multiple cycles and carries the running decompression
-- offset and long-term line pointer. Slot 1 generates a second command in the
-- same cycle, from slot 0's *advanced* offset/line-pointer, but only when the
-- lead partial command retires fully this cycle (no carried literal state) and
-- the next partial command is itself a single, self-contained command. The
-- flattened output (cm0 then cm1 where present) is identical to the single-issue
-- command sequence, which the standalone testbench checks against cm.tv.
--
-- 200 MHz restructure (dpu-smartssd-olap 20261005-decoder-timing-fix-01,
-- findings.md section 10). The original loaded the holding registers from the
-- input and processed them in the same cycle, and derived every command field
-- (17-bit relative offset, 12-bit line pointer sum, 11-bit even-line address)
-- inside the slot recurrence; on the xczu7ev proxy the c1_0h.valid recurrence
-- missed 200 MHz by 0.56 ns and the slot-1 line address by more. The same
-- command sequence is now produced by three stages:
--
--  * input: a one-entry skid register (ib) holds an accepted transfer, and the
--    slots are refilled at the end of a cycle from ib or from the transfer
--    accepted that cycle, so slot processing starts from registers only. The
--    per-slot facts slot processing needs (literal-length upper bits zero, slot
--    1 simple, its byte count) are computed at refill.
--  * stage A (the recurrence): advances the offset, line pointer and literal
--    state and decides advance/co-issue from a 5-bit compare instead of the
--    33-bit literal-length compare and subtract, and records for each emitted
--    command only the state it starts from (raw_command).
--  * stage B: derives the command_stream fields from the raw register exactly
--    as the original did (derive), so no field arithmetic sits in the
--    recurrence.
--
-- Commands reach cm two cycles later than before; the order, grouping into
-- {cm0, cm1} and every field are unchanged.
--
-- rd6 (decoder-timing-fix findings.md section 13). In the v09 full-design link
-- (run-v09-v2-rd5-01) this block failed by 0.173 ns: proc.s0_reg[cp_len] ->
-- proc.s1_b1_reg / proc.s1_simp_reg, 11-13 levels. The lead's copy length went
-- through the load_new mux, the literal budget compare, len0, b0 and the
-- co-issue sum b0 + s1_b1 <= C_BYTES into advance/fold and the refill decision
-- (fanout 98), and the refill then added s1_b1's adder or simple_cmd's compare
-- behind the source mux. The xczu7ev proxy showed the same structure as the
-- block's worst paths (+0.414 ns, c1_pend -> s1_simp, 13 levels; +0.488 ns,
-- c1_pend -> lt_ptr, 12 levels; rd-analysis/v09-hse_r4_rd5-01). Two changes,
-- both leaving every register value, and so the command sequence and its
-- timing, as before:
--
--  * lead_facts: what stage A derives from a partial command in the cycle it
--    starts as the lead (load_new = 1, the running literal has finished) depends
--    only on that command. It is computed where the command enters a slot (from
--    the producer's output register for a direct refill, or when the skid
--    register loads) and kept beside the slot, so stage A only selects between
--    it and the continuing-literal case, whose copy is empty and whose budget is
--    the constant C_BYTES; that case's "li_len < C_BYTES" test is a register
--    (li_small) updated with li_len.
--  * The slot-1 facts the refill derives (simple, byte count, b1) and the pair's
--    co-issue fit are computed the same way, in front of the source mux, so the
--    refill decision is followed by a 2:1 select only.
entity vhsnunzip_cmd_gen_2_dual is
  generic (
    LONG_CHUNKS : boolean := true
  );
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    c1          : in  dual_partial_command_stream;
    c1_ready    : out std_logic;

    lt_off_ld   : in  std_logic := '1';
    lt_off      : in  unsigned(C_AW downto 0) := (others => '0');

    cm          : out dual_command_stream;
    cm_ready    : in  std_logic
  );
end vhsnunzip_cmd_gen_2_dual;

architecture behavior of vhsnunzip_cmd_gen_2_dual is

  -- Whether a partial command retires in exactly one command: a copy-only or
  -- empty command, or one whose literal fits the remaining budget and whose
  -- literal data starts in the current line (li_off < C_BYTES). Last commands are
  -- never co-issued, to keep the end-of-chunk stall handling on slot 0.
  function simple_cmd(c : partial_command_stream) return boolean is
    variable budget : unsigned(C_CNT-1 downto 0);
  begin
    if c.last = '1' then
      return false;
    end if;
    budget := unsigned(c.cp_len(C_CNT-1 downto 0)) xor to_unsigned(C_BYTES-1, C_CNT);
    if c.li_val = '0' then
      return true;
    elsif c.li_off(C_IDX) = '1' then
      return false;
    elsif c.li_len < budget then     -- li_len + 1 <= budget
      return true;
    else
      return false;
    end if;
  end function;

  function li_high_fn return natural is
  begin
    if LONG_CHUNKS then
      return 32;
    else
      return 16;
    end if;
  end function;

  -- One emitted command as the state it starts from: the running offset and
  -- line pointer before it, its copy (cp_len diminished-one, -1 = none), and
  -- its literal start and byte count this cycle.
  type raw_command is record
    valid   : std_logic;
    off     : unsigned(C_CNT-1 downto 0);
    lt_ptr  : unsigned(C_AW downto 0);
    cp_off  : unsigned(15 downto 0);
    cp_len  : signed(C_CNT-1 downto 0);
    cp_rle  : std_logic;
    li_off  : unsigned(C_WIN-1 downto 0);
    li_cnt  : unsigned(C_CNT-1 downto 0);
    ld_pop  : std_logic;
    last    : std_logic;
  end record;

  constant RAW_COMMAND_INIT : raw_command := (
    valid   => '0',
    off     => (others => '0'),
    lt_ptr  => (others => '0'),
    cp_off  => (others => '0'),
    cp_len  => (others => '1'),
    cp_rle  => '0',
    li_off  => (others => '0'),
    li_cnt  => (others => '0'),
    ld_pop  => '0',
    last    => '0'
  );

  -- The command fields, computed as the original slot 0 / gen_simple did.
  function derive(r : raw_command) return command_stream is
    variable cmd    : command_stream;
    variable cp_rel : signed(16 downto 0);
    variable cp_lt  : unsigned(C_AW downto 0);
  begin
    cmd.valid  := r.valid;
    cmd.cp_rle := r.cp_rle;

    cp_rel := signed(resize(r.off, 17)) - signed(resize(r.cp_off, 17));
    cmd.st_addr := not unsigned(cp_rel(C_IDX+4 downto C_IDX));
    cp_lt := r.lt_ptr + unsigned(cp_rel(C_IDX+C_AW downto C_IDX));
    cmd.lt_swap := cp_lt(0);
    cmd.lt_adev := cp_lt(C_AW downto 1) + cp_lt(0 downto 0);
    cmd.lt_adod := cp_lt(C_AW downto 1);
    if cp_rel(16 downto C_IDX) < -31 then
      cmd.lt_val := not r.cp_len(C_CNT-1);
    else
      cmd.lt_val := '0';
    end if;

    if r.cp_rle = '1' then
      cmd.cp_rol := resize(unsigned(cp_rel(C_IDX-1 downto 0)), C_WIN);
    else
      cmd.cp_rol := unsigned(cp_rel(C_IDX-1 downto 0)) - r.off;
    end if;

    cmd.cp_end := r.off + unsigned(r.cp_len) + 1;
    cmd.li_rol := r.li_off - cmd.cp_end;
    cmd.li_end := cmd.cp_end + r.li_cnt;

    cmd.ld_pop := r.ld_pop;
    cmd.last   := r.last;
    return cmd;
  end function;

  -- Upper literal-length bits (li_high-1 .. C_CNT) of a slot command are zero.
  function li_upper_zero(c : partial_command_stream) return std_logic is
    variable v : signed(li_high_fn downto 0);
  begin
    v := signed(resize(c.li_len, li_high_fn + 1));
    if v(li_high_fn-1 downto C_CNT) = 0 then
      return '1';
    end if;
    return '0';
  end function;

  -- Literal byte count of a simple (single-command) partial command.
  function simple_li_cnt(c : partial_command_stream) return unsigned is
  begin
    if c.li_val = '1' then
      return c.li_len(C_CNT-1 downto 0) + 1;
    end if;
    return to_unsigned(0, C_CNT);
  end function;

  -- rd6: what stage A derives from a partial command in the cycle it starts as
  -- the lead (load_new = 1, so li_len = -1: a command without a literal leads
  -- with lil = -1), computed where the command enters a slot. Field by field it
  -- is the original stage A's value in that cycle: done, cross, len0 and b0, and
  -- small = li_small of the li_len stage A leaves behind.
  type lead_facts is record
    done  : std_logic;
    cross : std_logic;
    len0  : unsigned(C_CNT-1 downto 0);
    b0    : unsigned(C_CNT downto 0);
    small : std_logic;
  end record;

  constant LEAD_FACTS_INIT : lead_facts := (
    done  => '0',
    cross => '0',
    len0  => (others => '0'),
    b0    => (others => '0'),
    small => '0'
  );

  -- rd6: the facts of a transfer {c1_0, c1_1} that the refill used to derive
  -- behind the source mux: lead facts of both commands (c1_1 leads after a
  -- shift), slot 1's simple flag, byte count and b1, the co-issue fit of the pair
  -- in c1_0's first cycle (b0 + s1_b1 <= C_BYTES), and c1_pend after a refill.
  type pair_facts is record
    n0    : lead_facts;
    n1    : lead_facts;
    simp1 : std_logic;
    cnt1  : unsigned(C_CNT-1 downto 0);
    b1    : unsigned(C_CNT downto 0);
    fold  : std_logic;
    pend0 : std_logic;
  end record;

  constant PAIR_FACTS_INIT : pair_facts := (
    n0    => LEAD_FACTS_INIT,
    n1    => LEAD_FACTS_INIT,
    simp1 => '0',
    cnt1  => (others => '0'),
    b1    => (others => '0'),
    fold  => '0',
    pend0 => '0'
  );

  function lead(c : partial_command_stream) return lead_facts is
    variable f        : lead_facts;
    variable budget   : unsigned(C_CNT-1 downto 0);
    variable lil      : signed(li_high_fn downto 0);
    variable lzero    : std_logic;
    variable lt_small : std_logic;
    variable rest     : signed(li_high_fn downto 0);
  begin
    budget := unsigned(c.cp_len) xor to_unsigned(C_BYTES-1, C_CNT);
    if c.li_val = '1' then
      lil := signed(resize(c.li_len, li_high_fn + 1));
      lzero := li_upper_zero(c);
    else
      lil := (others => '1');
      lzero := '0';
    end if;
    if lil(lil'high) = '1' or (lzero = '1' and unsigned(lil(C_CNT-1 downto 0)) < budget) then
      lt_small := '1';
    else
      lt_small := '0';
    end if;
    if c.li_off(C_IDX) = '1' then
      f.len0 := (others => '0');
      f.done := lil(lil'high);
      f.cross := '1';
      rest := lil;
    else
      if lt_small = '1' then
        f.len0 := unsigned(lil(C_CNT-1 downto 0)) + 1;
      else
        f.len0 := budget;
      end if;
      f.done := lt_small;
      if resize(c.li_off, C_CNT+1) + resize(budget, C_CNT+1) >= C_BYTES then
        f.cross := '1';
      else
        f.cross := '0';
      end if;
      rest := lil - signed(resize(budget, li_high_fn + 1));
    end if;
    f.b0 := resize(unsigned(c.cp_len) + 1, C_CNT+1) + resize(f.len0, C_CNT+1);
    -- li_len after the cycle is -1 when done, else rest.
    if f.done = '1' or rest(rest'high) = '1' or rest(rest'high-1 downto C_IDX) = 0 then
      f.small := '1';
    else
      f.small := '0';
    end if;
    return f;
  end function;

  function pair(c : dual_partial_command_stream) return pair_facts is
    variable p : pair_facts;
  begin
    p.n0 := lead(c.c1_0);
    p.n1 := lead(c.c1_1);
    if simple_cmd(c.c1_1) then
      p.simp1 := '1';
    else
      p.simp1 := '0';
    end if;
    p.cnt1 := simple_li_cnt(c.c1_1);
    p.b1 := resize(unsigned(c.c1_1.cp_len) + 1, C_CNT+1) + resize(simple_li_cnt(c.c1_1), C_CNT+1);
    if p.n0.b0 + p.b1 <= C_BYTES then
      p.fold := '1';
    else
      p.fold := '0';
    end if;
    p.pend0 := (not c.c1_0.cp_len(C_CNT-1)) or c.c1_0.li_val;
    return p;
  end function;

begin
  proc: process (clk) is

    -- Input skid register, the two slots, and the facts computed where a
    -- transfer enters them (rd6: ibf with the skid register, s0n/s1n/s01_fold
    -- and the s1_* facts with the slots).
    variable ib       : dual_partial_command_stream := DUAL_PARTIAL_COMMAND_STREAM_INIT;
    variable ibf      : pair_facts := PAIR_FACTS_INIT;
    variable s0       : partial_command_stream := PARTIAL_COMMAND_STREAM_INIT;
    variable s1       : partial_command_stream := PARTIAL_COMMAND_STREAM_INIT;
    variable s0n      : lead_facts := LEAD_FACTS_INIT;
    variable s1n      : lead_facts := LEAD_FACTS_INIT;
    variable s01_fold : std_logic := '0';
    variable s1_simp  : std_logic := '0';
    variable s1_cnt   : unsigned(C_CNT-1 downto 0) := (others => '0');
    variable s1_b1    : unsigned(C_CNT downto 0) := (others => '0');
    variable c1_rdy   : std_logic := '0';

    -- Running decompression state (slot 0 owns the multi-cycle state).
    variable lt_val   : std_logic;
    variable lt_ptr   : unsigned(C_AW downto 0);
    variable c1_pend  : std_logic;
    variable li_len   : signed(li_high_fn downto 0) := (others => '1');
    -- rd6: li_len < C_BYTES or li_len = -1, i.e. the original's lt_small for a
    -- continuing literal (no copy, budget C_BYTES); written with li_len.
    variable li_small : std_logic := '1';
    variable li_off   : unsigned(C_WIN-1 downto 0);
    variable off      : unsigned(C_CNT-1 downto 0);

    -- Stage A temporaries.
    variable load_new : std_logic;
    variable pend     : std_logic;
    variable cpl      : signed(C_CNT-1 downto 0);
    variable lio      : unsigned(C_WIN-1 downto 0);
    variable c_len0   : unsigned(C_CNT-1 downto 0);
    variable c_done   : std_logic;
    variable c_rest   : signed(li_high_fn downto 0);
    variable c_small  : std_logic;
    variable c_fit    : std_logic;
    variable done     : std_logic;
    variable cross    : std_logic;
    variable len0     : unsigned(C_CNT-1 downto 0);
    variable b0       : unsigned(C_CNT downto 0);
    variable fit      : std_logic;
    variable sum      : unsigned(C_CNT downto 0);
    variable sum2     : unsigned(C_CNT downto 0);
    variable off1     : unsigned(C_CNT-1 downto 0);
    variable lt_ptr1  : unsigned(C_AW downto 0);
    variable advance  : boolean;
    variable fold     : boolean;

    -- Refill temporaries.
    variable acc      : boolean;
    variable src      : dual_partial_command_stream;
    variable srcf     : pair_facts;
    variable c1f      : pair_facts;
    variable take     : boolean;

    -- Stage A -> B raw register and stage B output register.
    variable cmr0     : raw_command := RAW_COMMAND_INIT;
    variable cmr1     : raw_command := RAW_COMMAND_INIT;
    variable cmh      : dual_command_stream := DUAL_COMMAND_STREAM_INIT;
    variable cmh_free : boolean;
    variable cmr_free : boolean;

    variable stall    : std_logic;

  begin
    if rising_edge(clk) then

      -- The producer saw c1_ready = c1_rdy this cycle, so a valid transfer is
      -- taken now (into a slot or the skid register below).
      acc := c1_rdy = '1' and c1.c1_0.valid = '1';

      -- Insert a stall cycle after the last transfer of a chunk (slot 0 only,
      -- as last commands are never co-issued).
      stall := cmh.cm0.valid and cm_ready and cmh.cm0.last;

      cmh_free := cmh.cm0.valid = '0' or cm_ready = '1';
      cmr_free := cmr0.valid = '0' or cmh_free;

      -- ================= Stage B: derive the command fields =================
      if cmh_free then
        cmh.cm0 := derive(cmr0);
        cmh.cm1 := derive(cmr1);
        cmh.cm1.valid := cmr0.valid and cmr1.valid;
        cmr0.valid := '0';
        cmr1.valid := '0';
      end if;

      -- ================= Stage A: slot 0, then co-issue of slot 1 ===========
      if s0.valid = '1' and cmr_free and stall = '0' then

        -- The lead command starts once the previous literal is done; until
        -- then slot 0 continues that literal (cp_len -1, carried li_len/li_off).
        load_new := li_len(li_len'high) and c1_pend;
        pend := c1_pend and not li_len(li_len'high);

        -- rd6: the continuing case (load_new = 0). The copy is empty (cp1 = 0) and
        -- the budget is C_BYTES, so the original's lt_small is li_small, len0 is
        -- li_len + 1 or C_BYTES, and cross is 1 (li_off + C_BYTES >= C_BYTES).
        if li_off(C_IDX) = '1' then
          c_len0 := (others => '0');
          c_done := li_len(li_len'high);
          c_rest := li_len;
          c_small := li_small;
        else
          if li_small = '1' then
            c_len0 := unsigned(li_len(C_CNT-1 downto 0)) + 1;
          else
            c_len0 := to_unsigned(C_BYTES, C_CNT);
          end if;
          c_done := li_small;
          c_rest := li_len - C_BYTES;
          -- When it does not finish, li_len >= C_BYTES, and li_len - C_BYTES <
          -- C_BYTES is li_len < 2 * C_BYTES.
          if li_len(li_len'high-1 downto C_IDX+1) = 0 then
            c_small := '1';
          else
            c_small := '0';
          end if;
        end if;
        -- Its co-issue fit b0 + s1_b1 <= C_BYTES matters only when c_done; then
        -- b0 = c_len0 is 0 (li_off(C_IDX) set, or li_len = -1) or li_len + 1 with
        -- 0 <= li_len < C_BYTES.
        if li_off(C_IDX) = '1' or li_len(li_len'high) = '1' then
          if s1_b1 <= C_BYTES then
            c_fit := '1';
          else
            c_fit := '0';
          end if;
        else
          if resize(unsigned(li_len(C_IDX-1 downto 0)), C_CNT+1) + s1_b1 < C_BYTES then
            c_fit := '1';
          else
            c_fit := '0';
          end if;
        end if;

        -- rd6: the lead case reads the facts held with slot 0 (lead_facts).
        if load_new = '1' then
          cpl := s0.cp_len;
          lio := s0.li_off;
          done := s0n.done;
          cross := s0n.cross;
          len0 := s0n.len0;
          b0 := s0n.b0;
          fit := s01_fold;
          -- The literal state after this cycle (li_len - len0 of the original);
          -- a lead without a literal is done.
          if s0n.done = '1' then
            li_len := (others => '1');
          elsif s0.li_off(C_IDX) = '1' then
            li_len := signed(resize(s0.li_len, li_len'length));
          else
            li_len := signed(resize(s0.li_len, li_len'length))
                      - signed(resize(unsigned(s0.cp_len) xor to_unsigned(C_BYTES-1, C_CNT), li_len'length));
          end if;
          li_small := s0n.small;
        else
          cpl := (others => '1');
          lio := li_off;
          done := c_done;
          cross := '1';
          len0 := c_len0;
          b0 := resize(c_len0, C_CNT+1);
          fit := c_fit;
          if c_done = '1' then
            li_len := (others => '1');
          else
            li_len := c_rest;
          end if;
          li_small := c_done or c_small;
        end if;

        -- Bytes slot 0 writes (at most C_BYTES), the advanced offset and line
        -- pointer, and the raw command.
        sum := resize(off, C_CNT+1) + b0;
        off1 := sum(C_CNT-1 downto 0);
        off1(C_IDX) := '0';
        if sum(C_IDX) = '1' then
          lt_ptr1 := lt_ptr + 1;
        else
          lt_ptr1 := lt_ptr;
        end if;

        cmr0.valid  := '1';
        cmr0.off    := off;
        cmr0.lt_ptr := lt_ptr;
        cmr0.cp_off := s0.cp_off;
        cmr0.cp_len := cpl;
        cmr0.cp_rle := s0.cp_rle;
        cmr0.li_off := lio;
        cmr0.li_cnt := len0;
        cmr1.valid  := '0';

        li_off := lio + len0;
        off := off1;
        lt_ptr := lt_ptr1;
        c1_pend := pend;

        advance := pend = '0' and (done = '1' or (s0.last = '0' and cross = '1'));

        if advance then
          cmr0.ld_pop := s0.ld_pop;
          cmr0.last   := s0.last;
          li_off := li_off - C_BYTES;
          if s0.last = '1' then
            lt_val := '0';
            off := (others => '0');
          end if;

          -- Slot 1: co-issue when the lead retired fully, is not last, the
          -- lookahead is simple, and the pair writes at most one line (rd6:
          -- fit = b0 + s1_b1 <= C_BYTES, precomputed per case above).
          fold := done = '1' and s0.last = '0' and s1.valid = '1' and s1_simp = '1'
                  and fit = '1';
          if fold then
            cmr1.valid  := '1';
            cmr1.off    := off1;
            cmr1.lt_ptr := lt_ptr1;
            cmr1.cp_off := s1.cp_off;
            cmr1.cp_len := s1.cp_len;
            cmr1.cp_rle := s1.cp_rle;
            cmr1.li_off := s1.li_off;
            cmr1.li_cnt := s1_cnt;
            cmr1.ld_pop := s1.ld_pop;
            cmr1.last   := s1.last;
            sum2 := resize(off1, C_CNT+1) + s1_b1;
            off := sum2(C_CNT-1 downto 0);
            off(C_IDX) := '0';
            if sum2(C_IDX) = '1' then
              lt_ptr := lt_ptr1 + 1;
            end if;
            s0.valid := '0';
            s1.valid := '0';
          else
            -- Shift the lookahead into the lead slot (deferred to its own
            -- cycle, or keeping any carried literal state).
            s0 := s1;
            s0n := s1n;
            if s1.valid = '1' then
              c1_pend := (not s1.cp_len(C_CNT-1)) or s1.li_val;
            end if;
            s1.valid := '0';
          end if;
        else
          cmr0.ld_pop := '0';
          cmr0.last   := '0';
        end if;

      end if;

      -- Load the long-term memory pointer when we get it.
      if lt_val = '0' and lt_off_ld = '1' then
        lt_ptr := lt_off;
        lt_val := '1';
      end if;

      -- ================= Refill the slots from registers ====================
      -- rd6: the facts of the producer's transfer, for a direct refill and for
      -- the skid register (pair_facts).
      c1f := pair(c1);
      take := false;
      if s0.valid = '0' and s1.valid = '0' and lt_val = '1' then
        if ib.c1_0.valid = '1' then
          src := ib;
          srcf := ibf;
          ib.c1_0.valid := '0';
          take := true;
        elsif acc then
          src := c1;
          srcf := c1f;
          acc := false;
          take := true;
        end if;
      end if;
      if take then
        s0 := src.c1_0;
        s1 := src.c1_1;
        s0n := srcf.n0;
        s1n := srcf.n1;
        s01_fold := srcf.fold;
        s1_simp := srcf.simp1;
        s1_cnt := srcf.cnt1;
        s1_b1 := srcf.b1;
        c1_pend := srcf.pend0;
      end if;
      if acc then
        ib := c1;
        ibf := c1f;
      end if;

      -- Handle reset.
      if reset = '1' then
        ib.c1_0.valid := '0';
        s0.valid := '0';
        s1.valid := '0';
        cmr0.valid := '0';
        cmr1.valid := '0';
        cmh.cm0.valid := '0';
        cmh.cm1.valid := '0';
        lt_val := '0';
        c1_pend := '0';
        li_len := (others => '1');
        li_small := '1';
        off := (others => '0');
      end if;

      -- Assign outputs. c1_ready is registered: the skid register is free.
      c1_rdy := not ib.c1_0.valid;
      if reset = '1' then
        c1_rdy := '0';
      end if;
      c1_ready <= c1_rdy;
      cm <= cmh;

    end if;
  end process;
end behavior;
