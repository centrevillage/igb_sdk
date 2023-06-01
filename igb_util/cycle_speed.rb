require 'bigdecimal'

sampling_freq = 48000
scaling = 20
table_size = 12 * 100
base_freq = BigDecimal("440") * (BigDecimal("2.0") ** BigDecimal(BigDecimal(3)/BigDecimal(12)))
table_size.times.each do |i|
  hz = base_freq * (BigDecimal("2.0") ** BigDecimal(BigDecimal(i)/BigDecimal(table_size)))
  puts "#{(hz * (2**scaling) / sampling_freq).round},"
end
