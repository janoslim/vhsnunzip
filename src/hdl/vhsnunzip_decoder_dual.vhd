library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library work;
use work.vhsnunzip_utils_pkg.all;
use work.vhsnunzip_int_pkg.all;

-- Speculative dual-issue Snappy element decoder.
--
-- This decodes two elements per cycle. It mirrors the single-issue decoder
-- (vhsnunzip_decoder, the <=64kiB variant): element 0 is decoded at the current
-- offset, and a second element is emitted when element 0 does not cross the line
-- boundary (so element 1's header is inside the current two-line window).
--
-- The naive way to find element 1 is to decode it at element 0's *end* offset,
-- i.e. chain a second decode onto the combinational output of the first. That
-- chain, together with the data-byte mux that selects the header bytes at the
-- loop-carried offset `off`, is the timing bottleneck: the byte select + header
-- decode + end-offset arithmetic all sit inside the single-cycle offset
-- recurrence.
--
-- We take both off the recurrence by pre-decoding the *whole line* one stage
-- early. Every element starts at a line-local offset 0 .. C_BYTES-1, so we run
-- C_BYTES copies of the per-element decode -- one per possible start offset, each
-- reading a *fixed* set of header bytes (no off-driven byte mux) -- and register
-- the resulting `decoded_t` array. Because a decode at offset j only ever
-- indexes data modulo the line width, decoding at offset j is exactly the result
-- a chained decode starting at j would produce. Element 0 is then simply
-- `dec(off)` and element 1 is `dec(off0)` (off0 = element 0's end offset): the
-- recurrence degenerates to two cascaded record muxes over the registered array
-- plus the narrow `off` arithmetic, with no header decode in the loop at all.
--
-- Pipeline (all three are 1-deep elastic, so steady-state throughput is one
-- transfer (up to two elements) per cycle; only latency grows):
--
--   Decode-all stage (cd -> cdd): a depth-2 circular FIFO that registers each
--   incoming double line together with its full per-offset decode `cdd_dec`. The
--   C_BYTES parallel header decoders live here, between two registers (cd ..
--   cdd_dec), off the recurrence. Two slots (rather than one) let the FIFO drain
--   into the holding register and refill from `cd` on the *same* cycle, which is
--   what covers the one-cycle latency of the registered cd_ready and sustains one
--   line/cycle into the recurrence (e.g. the long-literal pop path). The line is
--   pre-fetched/decoded while it waits, so the holding register below refills
--   from an *already decoded* line with no bubble.
--
--   Stage 1 (recurrence): loads the holding register `cdh`/`cdh_dec` from the
--   decode-all stage, then advances the loop-carried offset `off` and selects
--   element 0 = cdh_dec(off) and element 1 = cdh_dec(off0). Only the (registered)
--   array is read here, so the loop carries no decode logic.
--
--   Stage 2 (format): builds the dual_element_stream output from the stage-1
--   register `s1`, which now carries the two selected decodes directly.
--
-- SPEC_OFFSETS bounds element 0's size for which element 1 is co-issued: when
-- element 0 is larger than SPEC_OFFSETS+1 bytes we emit element 0 alone and pick
-- element 1 up as next cycle's element 0 -- always correct, just not dual-issued
-- that cycle. SPEC_OFFSETS = C_BYTES-2 covers every reachable element-0 size
-- (then behaviourally identical to the single-issue decoder's element sequence);
-- SPEC_OFFSETS = 0 disables co-issue entirely. The standalone testbench checks
-- the flattened (el0, then el1 when present) stream against the single-issue
-- el.tv, which holds for any SPEC_OFFSETS.
--
-- 200 MHz restructure (dpu-smartssd-olap 20261005-decoder-timing-fix-01,
-- findings.md section 10). On the xczu7ev proxy the offset recurrence missed
-- 200 MHz by 0.49 ns starting at `count` (the holding-register load decision,
-- whose select fans out to the whole cdh/cdh_dec copy), and the decoders' input
-- path by 0.27 ns (4 logic levels, the rest routing from the pre-decoder's
-- register). The element sequence is unchanged; the structure is now:
--
--  * cd is accepted into a raw register cdi, and the decoders read cdi; cdi plus
--    the 2-slot FIFO hold up to 3 lines (as the FIFO plus cdh did before).
--  * there is no holding register: stage 1 reads the FIFO head cdd(rp) and its
--    decode directly and pops it where cdh.valid used to clear. A line that
--    becomes the head applies first/start at the end of that cycle.
--  * at full coverage (SPEC_OFFSETS >= C_BYTES-2) the element-0 size test is
--    always true for reachable states (comment at the test) and is dropped.
--  * each decode slot also carries, per start offset, whether that element ends
--    past the line (off_next > endi) and off_next - C_BYTES, computed by the
--    decoders, so the recurrence has no compare or subtract after its lookups.
--  * the decode-all stage writes a slot only if one was free at the start of the
--    cycle, so its write enable (the CE of every cdd_dec bit) does not wait for
--    stage 1's pop decision (hse_r1 proxy: off -> cdd_dec CE, 12 levels,
--    +0.030 ns). A slot freed by this cycle's pop is refilled next cycle, while
--    stage 1 already works on the other slot, so a line still reaches stage 1
--    every cycle when stage 1 pops every cycle.
--
-- A line now reaches stage 1 one cycle later than before (through cdi).
entity vhsnunzip_decoder_dual is
  generic (

    -- Maximum element-0 size (in bytes, minus one) for which element 1 is
    -- co-issued. 0 disables dual-issue (single element per cycle). See
    -- C_SPEC_OFFSETS in vhsnunzip_utils_pkg.
    SPEC_OFFSETS : natural := C_SPEC_OFFSETS

  );
  port (
    clk         : in  std_logic;
    reset       : in  std_logic;

    -- Double line compressed data stream input.
    cd          : in  compressed_stream_double;
    cd_ready    : out std_logic;

    -- Dual element information stream output.
    el          : out dual_element_stream;
    el_ready    : in  std_logic
  );
end vhsnunzip_decoder_dual;

architecture behavior of vhsnunzip_decoder_dual is

  -- Result of decoding one element at a given offset.
  type decoded_t is record
    cp_val   : std_logic;
    cp_off   : unsigned(15 downto 0);
    cp_len   : unsigned(5 downto 0);
    li_val   : std_logic;
    li_off   : unsigned(C_WIN-1 downto 0);
    li_len   : unsigned(31 downto 0);
    off_next : unsigned(16 downto 0);
  end record;

  constant DECODED_INIT : decoded_t := (
    cp_val   => '0',
    cp_off   => (others => '0'),
    cp_len   => (others => '0'),
    li_val   => '0',
    li_off   => (others => '0'),
    li_len   => (others => '0'),
    off_next => (others => '0')
  );

  type decoded_array is array (natural range <>) of decoded_t;

  -- Smallest possible element size in bytes (a 2-byte copy, or a 1-byte literal
  -- header plus a single data byte). Element 1 therefore starts at least this
  -- many bytes past element 0, so element-0 sizes below this never occur.
  constant DELTA_MIN : natural := 2;

  -- Stage-1 -> stage-2 pipeline register. Carries the two selected decodes and
  -- the per-slot control bits; the element_stream formatting happens in stage 2.
  type s1_reg_t is record
    valid   : std_logic;                 -- stage produced a transfer
    empty0  : std_logic;                 -- el0 is the empty literal-continuation
    d0      : decoded_t;                  -- el0 decode (when empty0 = '0')
    e0_pop  : std_logic;                 -- el0 ld_pop
    e0_last : std_logic;                 -- el0 last
    e1_val  : std_logic;                 -- el1 present
    e1_pop  : std_logic;                 -- el1 ld_pop
    e1_last : std_logic;                 -- el1 last
    d1      : decoded_t;                  -- el1 decode (when e1_val = '1')
  end record;

  constant S1_REG_INIT : s1_reg_t := (
    valid   => '0',
    empty0  => '0',
    d0      => DECODED_INIT,
    e0_pop  => '0',
    e0_last => '0',
    e1_val  => '0',
    e1_pop  => '0',
    e1_last => '0',
    d1      => DECODED_INIT
  );

  -- Decode a single element starting at byte offset `off` within the line
  -- window `data`, given the last valid byte index `endi`. Assumes off <= endi
  -- (the caller handles the literal-data continuation case where off > endi).
  -- Returns the element header info and the offset just past this element's
  -- data. This is the same logic as vhsnunzip_decoder's per-element decode.
  function decode_one(
    data : byte_array;
    off  : unsigned;
    endi : unsigned
  ) return decoded_t is
    variable r     : decoded_t;
    variable ofi   : integer;
    variable offns : unsigned(C_WIN-1 downto 0);
  begin
    r.cp_val := '0';
    r.li_val := '0';
    r.cp_off := (others => '0');
    r.cp_len := (others => '0');
    r.li_off := (others => '0');
    r.li_len := (others => '0');

    -- Copy element.
    offns := resize(off(C_IDX-1 downto 0), C_WIN);
    ofi := to_integer(off(C_IDX-1 downto 0));

    case data(ofi)(1 downto 0) is
      when "01" => r.cp_val := '1'; offns := offns + 2;
      when "10" => r.cp_val := '1'; offns := offns + 3;
      when "11" => r.cp_val := '1'; offns := offns + 5;
      when others => r.cp_val := '0';
    end case;

    if data(ofi)(1) = '0' then
      r.cp_off := "00000" & unsigned(data(ofi)(7 downto 5)) & unsigned(data(ofi + 1));
      r.cp_len := resize(unsigned(data(ofi)(4 downto 2)), 6) + 3;
    else
      r.cp_off := unsigned(data(ofi + 2)) & unsigned(data(ofi + 1));
      r.cp_len := unsigned(data(ofi)(7 downto 2));
    end if;

    -- Literal element.
    ofi := to_integer(offns(C_IDX-1 downto 0));

    if offns > endi then
      r.li_val := '0';
    elsif data(ofi)(1 downto 0) /= "00" then
      r.li_val := '0';
    elsif data(ofi)(7 downto 4) = "1111" then
      r.li_val := '1';
      offns := offns + 2 + unsigned(data(ofi)(3 downto 2));
    else
      r.li_val := '1';
      offns := offns + 1;
    end if;

    r.li_off := offns;

    if std_match(data(ofi), "111100--") then
      r.li_len := X"000000" & unsigned(data(ofi + 1));
    elsif std_match(data(ofi), "1111----") then
      r.li_len := X"0000" & unsigned(data(ofi + 2)) & unsigned(data(ofi + 1));
    else
      r.li_len := X"000000" & "00" & unsigned(data(ofi)(7 downto 2));
    end if;

    -- Seek past literal data.
    r.off_next := resize(offns, 17);
    if r.li_val = '1' then
      r.off_next := r.off_next + resize(r.li_len, 17) + 1;
    end if;

    return r;
  end function;

  -- Copy a decoded element's header fields into an element_stream record.
  procedure apply(variable e : inout element_stream; constant d : in decoded_t) is
  begin
    e.cp_val := d.cp_val;
    e.cp_off := d.cp_off;
    e.cp_len := d.cp_len;
    e.li_val := d.li_val;
    e.li_off := d.li_off;
    e.li_len := d.li_len;
  end procedure;

begin
  proc: process (clk) is

    -- Decode-all stage (cd -> cdi -> cdd): a depth-2 circular FIFO of decoded
    -- double lines, each registered together with its full per-offset decode,
    -- fed through the raw input register cdi. The C_BYTES header decoders sit
    -- between cdi and cdd_dec(wp) (200 MHz restructure, see the header: the
    -- decoders used to read the pre-decoder's register directly, a 4-level path
    -- of 5.3 ns on the xczu7ev proxy that was almost all fanout routing).
    constant CDD_DEPTH : natural := 2;
    type cdd_array_t is array (natural range <>) of compressed_stream_double;
    type cdd_dec_array_t is array (natural range <>) of decoded_array(0 to C_BYTES-1);
    variable cdd     : cdd_array_t(0 to CDD_DEPTH-1) :=
        (others => COMPRESSED_STREAM_DOUBLE_INIT);
    variable cdd_dec : cdd_dec_array_t(0 to CDD_DEPTH-1) :=
        (others => (others => DECODED_INIT));

    -- Per slot and start offset: the element crosses the line (off_next > endi),
    -- and its next offset relative to the following line (off_next - C_BYTES).
    type cross_array_t is array (natural range <>) of std_logic_vector(0 to C_BYTES-1);
    type next_array_t is array (natural range <>) of unsigned(16 downto 0);
    type next_slot_array_t is array (natural range <>) of next_array_t(0 to C_BYTES-1);
    variable cdd_cr  : cross_array_t(0 to CDD_DEPTH-1) := (others => (others => '0'));
    variable cdd_om  : next_slot_array_t(0 to CDD_DEPTH-1) := (others => (others => (others => '0')));
    variable dj      : decoded_t;

    -- Circular-FIFO read/write pointers (1-bit; wrap on +1 for CDD_DEPTH=2) and
    -- occupancy count. The pointer arithmetic below assumes a power-of-two depth.
    variable rp      : unsigned(0 downto 0) := (others => '0');
    variable wp      : unsigned(0 downto 0) := (others => '0');
    variable count   : natural range 0 to CDD_DEPTH := 0;

    -- Raw input register (one line ahead of the decoders).
    variable cdi     : compressed_stream_double := COMPRESSED_STREAM_DOUBLE_INIT;

    -- Mirror of the registered cd_ready output: the room-available value the
    -- producer is currently observing (set last cycle from cdi + count). The
    -- accept must gate on *this*, not on live occupancy -- because cd_ready is
    -- registered, the producer holds cd valid until it sees the ready one cycle
    -- later, so gating on live occupancy would re-accept the same line.
    variable cd_rdy  : std_logic := '0';

    -- Stage 1 reads the FIFO head directly (no holding register in front of the
    -- recurrence): hd is cdd(rp), consumed when stage 1 is done with the line.
    variable hd       : compressed_stream_double;
    variable consumed : boolean;
    variable was_empty : boolean;
    variable count0    : natural range 0 to CDD_DEPTH;

    -- Offset of the next element with respect to the head line's data
    -- (loop-carried).
    variable off    : unsigned(16 downto 0) := (others => '0');

    -- Element 0 / element 1 selected decodes and element 0's end offset.
    variable d0     : decoded_t;
    variable d1     : decoded_t;
    variable off0   : unsigned(16 downto 0);
    variable esize  : unsigned(16 downto 0);
    variable sidx   : integer;

    -- Stage-1 -> stage-2 pipeline register.
    variable s1     : s1_reg_t := S1_REG_INIT;

    -- Stage-2 output holding register.
    variable elo    : dual_element_stream := DUAL_ELEMENT_STREAM_INIT;

    -- Flow-control: stage slot free for new data this cycle.
    variable s2_free : boolean;
    variable s1_free : boolean;

  begin
    if rising_edge(clk) then

      -- Flow-control decisions from the entry state. A stage slot is free when
      -- it is empty or its content moves downstream this cycle.
      s2_free := (elo.el0.valid = '0') or (el_ready = '1');
      s1_free := (s1.valid = '0') or s2_free;

      -- ================= Stage 2: format output from s1 =================
      -- Reads only the two registered decodes carried in s1.
      if s2_free then
        if s1.valid = '1' then

          -- Element 0.
          if s1.empty0 = '1' then
            elo.el0 := ELEMENT_STREAM_INIT;
            elo.el0.cp_val := '0';
            elo.el0.li_val := '0';
          else
            apply(elo.el0, s1.d0);
          end if;
          elo.el0.ld_pop := s1.e0_pop;
          elo.el0.last   := s1.e0_last;
          elo.el0.valid  := '1';

          -- Element 1.
          if s1.e1_val = '1' then
            apply(elo.el1, s1.d1);
            elo.el1.ld_pop := s1.e1_pop;
            elo.el1.last   := s1.e1_last;
            elo.el1.valid  := '1';
          else
            elo.el1.valid  := '0';
          end if;

        else
          elo.el0.valid := '0';
          elo.el1.valid := '0';
        end if;
      end if;

      -- ================= Stage 1: recurrence over the FIFO head ==============
      -- The head line hd = cdd(rp) and its decode cdd_dec(rp) are registers, so
      -- the recurrence starts from registers (the original first moved the head
      -- into a holding register in the same cycle, with a fanout-heavy select
      -- derived from count in front of the offset arithmetic). A line is
      -- consumed when the original cleared cdh.valid. The head slot is always
      -- one written in an earlier cycle.
      consumed := false;
      was_empty := count = 0;
      count0 := count;
      if s1_free then

        if count > 0 then
          hd := cdd(to_integer(rp));

          if off > hd.endi then

            -- Literal-data continuation: the whole line is part of a previously
            -- decoded literal. Emit an empty element that pops the line.
            s1.valid  := '1';
            s1.empty0 := '1';
            s1.e0_pop := '1';
            s1.e0_last := hd.last;
            s1.e1_val := '0';
            off := off - C_BYTES;
            consumed := true;

          else

            -- Element 0 is the registered decode at the current offset; its end
            -- offset is element 1's start offset. No header decode happens here.
            d0   := cdd_dec(to_integer(rp))(to_integer(off(C_IDX-1 downto 0)));
            off0 := d0.off_next;

            s1.valid  := '1';
            s1.empty0 := '0';
            s1.d0     := d0;

            if cdd_cr(to_integer(rp))(to_integer(off(C_IDX-1 downto 0))) = '1' then

              -- Element 0 crosses the line; emit only element 0 and pull a new
              -- line. This is the single-issue case.
              s1.e0_pop  := '1';
              s1.e0_last := hd.last;
              s1.e1_val  := '0';
              off := cdd_om(to_integer(rp))(to_integer(off(C_IDX-1 downto 0)));
              consumed := true;

            else

              -- Element 0 stays within the line, so element 1's header is in the
              -- window. Element 1 is just the registered decode at off0, co-issued
              -- when element 0 is small enough (size <= SPEC_OFFSETS+1).
              s1.e0_pop  := '0';
              s1.e0_last := '0';

              esize := off0 - off;                  -- element 0 size (>= DELTA_MIN)
              sidx  := to_integer(esize) - DELTA_MIN;

              -- Here off <= endi and off0 <= endi <= C_BYTES-1, and decode_one
              -- ends every element at least DELTA_MIN bytes after it starts, so
              -- esize is in DELTA_MIN .. C_BYTES-1 and sidx < C_BYTES-2: at full
              -- coverage the size test always passes, and is dropped rather than
              -- left as two carry chains in the recurrence.
              if SPEC_OFFSETS >= C_BYTES - 2 or (sidx >= 0 and sidx < SPEC_OFFSETS) then

                d1 := cdd_dec(to_integer(rp))(to_integer(off0(C_IDX-1 downto 0)));
                s1.e1_val := '1';
                s1.d1     := d1;

                if cdd_cr(to_integer(rp))(to_integer(off0(C_IDX-1 downto 0))) = '1' then
                  s1.e1_pop  := '1';
                  s1.e1_last := hd.last;
                  off := cdd_om(to_integer(rp))(to_integer(off0(C_IDX-1 downto 0)));
                  consumed := true;
                else
                  s1.e1_pop  := '0';
                  s1.e1_last := '0';
                  off := d1.off_next;
                end if;

              else

                -- Element 0's size is outside the co-issue range: emit element 0
                -- only and keep the line. Element 1 becomes next cycle's element
                -- 0 (functionally identical, just not dual-issued this cycle).
                s1.e1_val := '0';
                off := off0;

              end if;

            end if;

          end if;

        else
          s1.valid := '0';
        end if;

      end if;
      if consumed then
        rp    := rp + 1;
        count := count - 1;
      end if;

      -- ================= Decode-all stage (cdi -> cdd(wp)) ==================
      -- Decode the raw input line into the FIFO tail slot whenever there is
      -- room. The C_BYTES parallel header decoders sit between cdi and
      -- cdd_dec(wp), both registered here; each decodes at a fixed start offset,
      -- so there is no off-driven byte mux.
      if cdi.valid = '1' and count0 < CDD_DEPTH then
        cdd(to_integer(wp)) := cdi;
        for j in 0 to C_BYTES-1 loop
          dj := decode_one(cdi.data, to_unsigned(j, off'length), cdi.endi);
          cdd_dec(to_integer(wp))(j) := dj;
          if dj.off_next > cdi.endi then
            cdd_cr(to_integer(wp))(j) := '1';
          else
            cdd_cr(to_integer(wp))(j) := '0';
          end if;
          cdd_om(to_integer(wp))(j) := dj.off_next - C_BYTES;
        end loop;
        wp    := wp + 1;
        count := count + 1;
        cdi.valid := '0';
      end if;

      -- ================= Input (cd -> cdi) ==================================
      -- Gated on the presented cd_rdy (not live occupancy): cd_rdy = '1' means
      -- cdi plus the FIFO held at most CDD_DEPTH lines at last cycle end, so cdi
      -- is free here (it was empty, or it moved into a slot above).
      if cd.valid = '1' and cd_rdy = '1' then
        cdi := cd;
      end if;

      -- A line that just became the head starts its chunk at its start offset
      -- (the original applied this when loading the holding register).
      if (consumed or was_empty) and count > 0 then
        if cdd(to_integer(rp)).first = '1' then
          off := resize(cdd(to_integer(rp)).start, 17);
        end if;
      end if;

      -- Handle reset.
      if reset = '1' then
        rp    := (others => '0');
        wp    := (others => '0');
        count := 0;
        cdi.valid := '0';
        cd_rdy := '0';
        s1.valid  := '0';
        elo.el0.valid := '0';
        elo.el1.valid := '0';
        off := (others => '0');
      end if;

      -- Assign outputs. cd_ready is registered (room for another line in cdi
      -- plus the FIFO); cd_rdy holds the same value for next cycle's accept.
      if count < CDD_DEPTH or (count = CDD_DEPTH and cdi.valid = '0') then
        cd_rdy := '1';
      else
        cd_rdy := '0';
      end if;
      cd_ready <= cd_rdy;
      el <= elo;

    end if;
  end process;
end behavior;
