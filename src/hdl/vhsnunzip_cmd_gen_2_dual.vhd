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

begin
  proc: process (clk) is

    -- Input skid register, the two slots, and the facts computed at refill.
    variable ib       : dual_partial_command_stream := DUAL_PARTIAL_COMMAND_STREAM_INIT;
    variable s0       : partial_command_stream := PARTIAL_COMMAND_STREAM_INIT;
    variable s1       : partial_command_stream := PARTIAL_COMMAND_STREAM_INIT;
    variable s0_lz    : std_logic := '0';
    variable s1_lz    : std_logic := '0';
    variable s1_simp  : std_logic := '0';
    variable s1_cnt   : unsigned(C_CNT-1 downto 0) := (others => '0');
    variable s1_b1    : unsigned(C_CNT downto 0) := (others => '0');
    variable c1_rdy   : std_logic := '0';

    -- Running decompression state (slot 0 owns the multi-cycle state).
    variable lt_val   : std_logic;
    variable lt_ptr   : unsigned(C_AW downto 0);
    variable c1_pend  : std_logic;
    variable li_len   : signed(li_high_fn downto 0) := (others => '1');
    variable li_off   : unsigned(C_WIN-1 downto 0);
    variable off      : unsigned(C_CNT-1 downto 0);

    -- Stage A temporaries.
    variable load_new : std_logic;
    variable pend     : std_logic;
    variable cpl      : signed(C_CNT-1 downto 0);
    variable lil      : signed(li_high_fn downto 0);
    variable lio      : unsigned(C_WIN-1 downto 0);
    variable lzero    : std_logic;
    variable budget   : unsigned(C_CNT-1 downto 0);
    variable cp1      : unsigned(C_CNT-1 downto 0);
    variable lp1      : unsigned(C_CNT-1 downto 0);
    variable lt_small : std_logic;
    variable done     : std_logic;
    variable cross    : std_logic;
    variable len0     : unsigned(C_CNT-1 downto 0);
    variable b0       : unsigned(C_CNT downto 0);
    variable sum      : unsigned(C_CNT downto 0);
    variable sum2     : unsigned(C_CNT downto 0);
    variable off1     : unsigned(C_CNT-1 downto 0);
    variable lt_ptr1  : unsigned(C_AW downto 0);
    variable advance  : boolean;
    variable fold     : boolean;

    -- Refill temporaries.
    variable acc      : boolean;
    variable src      : dual_partial_command_stream;
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
        if load_new = '1' then
          cpl := s0.cp_len;
          lio := s0.li_off;
          if s0.li_val = '1' then
            lil := signed(resize(s0.li_len, li_len'length));
            lzero := s0_lz;
          else
            lil := li_len;
            lzero := '0';
          end if;
        else
          cpl := (others => '1');
          lio := li_off;
          lil := li_len;
          if li_len(li_len'high-1 downto C_CNT) = 0 then
            lzero := '1';
          else
            lzero := '0';
          end if;
        end if;

        -- Literal bytes this cycle. li_len < budget (li_len'high-bit compare in
        -- the original) is: negative, or upper bits zero and the 5-bit compare.
        budget := unsigned(cpl) xor to_unsigned(C_BYTES-1, C_CNT);
        cp1 := unsigned(cpl) + 1;
        lp1 := unsigned(lil(C_CNT-1 downto 0)) + 1;
        if lil(lil'high) = '1' or (lzero = '1' and unsigned(lil(C_CNT-1 downto 0)) < budget) then
          lt_small := '1';
        else
          lt_small := '0';
        end if;
        if lio(C_IDX) = '1' then
          len0 := (others => '0');
          done := lil(lil'high);
          cross := '1';
        else
          if lt_small = '1' then
            len0 := lp1;
          else
            len0 := budget;
          end if;
          done := lt_small;
          if resize(lio, C_CNT+1) + resize(budget, C_CNT+1) >= C_BYTES then
            cross := '1';
          else
            cross := '0';
          end if;
        end if;

        -- Bytes slot 0 writes (at most C_BYTES), the advanced offset and line
        -- pointer, and the raw command.
        b0 := resize(cp1, C_CNT+1) + resize(len0, C_CNT+1);
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

        -- The literal state after this cycle (li_len - len0 of the original).
        if done = '1' then
          li_len := (others => '1');
        elsif lio(C_IDX) = '1' then
          li_len := lil;
        else
          li_len := lil - signed(resize(budget, li_len'length));
        end if;
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
          -- lookahead is simple, and the pair writes at most one line.
          fold := done = '1' and s0.last = '0' and s1.valid = '1' and s1_simp = '1'
                  and b0 + s1_b1 <= C_BYTES;
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
            s0_lz := s1_lz;
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
      take := false;
      if s0.valid = '0' and s1.valid = '0' and lt_val = '1' then
        if ib.c1_0.valid = '1' then
          src := ib;
          ib.c1_0.valid := '0';
          take := true;
        elsif acc then
          src := c1;
          acc := false;
          take := true;
        end if;
      end if;
      if take then
        s0 := src.c1_0;
        s1 := src.c1_1;
        s0_lz := li_upper_zero(src.c1_0);
        s1_lz := li_upper_zero(src.c1_1);
        if simple_cmd(src.c1_1) then
          s1_simp := '1';
        else
          s1_simp := '0';
        end if;
        s1_cnt := simple_li_cnt(src.c1_1);
        s1_b1 := resize(unsigned(src.c1_1.cp_len) + 1, C_CNT+1) + resize(simple_li_cnt(src.c1_1), C_CNT+1);
        c1_pend := (not src.c1_0.cp_len(C_CNT-1)) or src.c1_0.li_val;
      end if;
      if acc then
        ib := c1;
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
