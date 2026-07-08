# LilaC issue #201 (Phase C): generator for the v01 -> prewarped-g cutoff LUTs
# in state_variable_filter.hpp. Offline literals (pitch_semitone_lut precedent):
# bit-identical on host and device (no reliance on constexpr libm folding,
# which differs between toolchains), .rodata-resident, and unit-testable
# against the closed form.
#
#   fc(v01) = 20 * 900^v01            (exponential 20 Hz .. 18 kHz)
#   g(v01)  = tan(pi * fc(v01) / fs)  (TPT/bilinear prewarp)
#
# Usage: ruby svf_cutoff_lut_gen.rb   (paste the output into the header)

table_last = 128  # 129 entries, read with lerp over 128 intervals

[48000, 96000].each do |fs|
  puts "constexpr float svf_g_tbl_#{fs / 1000}k[svf_g_tbl_last + 1] = {"
  (0..table_last).each_slice(4) do |idxs|
    row = idxs.map do |i|
      v01 = i.to_f / table_last
      fc = 20.0 * (900.0**v01)
      g = Math.tan(Math::PI * fc / fs)
      # 9 significant digits: exact float32 round-trip on both toolchains.
      format('%.9gf,', g)
    end
    puts '  ' + row.join(' ')
  end
  puts '};'
  puts
end
