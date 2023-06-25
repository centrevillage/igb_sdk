#require 'gruff'

SAMPLE_SIZE = 1024

def sigmoid(x, a=4)
  sv = 1.0 / (1.0 + Math.exp(-a * -1.0))
  ev = 1.0 / (1.0 + Math.exp(-a * 1.0))
  x = x * 2.0 - 1.0
  v = 1.0 / (1.0 + Math.exp(-a * x))
  ((v - sv) / (ev - sv)) * 2.0 - 1.0
end

def sinusoid(x)
  Math.sin(2.0*Math::PI*x)
end

def log(x)
  n = 10.0
  Math.log(x*(n - 1.0) + 1.0, n)
end

# def exp(x)
#   (Math.exp(x)-1.0) / (Math::E-1.0)
# end

def create_table_cpp_file
  open('dsp_tbl_func.cpp', 'w') do |file|
    file.puts <<-EOS
#include <igb_util/dsp_tbl_func.hpp>

    EOS
    file.puts <<-EOS
const float dsp_func_sigmoid_tbl[] = {
    EOS
    SAMPLE_SIZE.times do |i|
      file.puts <<-EOS
  #{'%.20f' % sigmoid(i.to_f / (SAMPLE_SIZE-1).to_f)},
      EOS
    end
    file.puts <<-EOS
};
    EOS
    file.puts <<-EOS
const float dsp_func_sinusoid_tbl[] = {
    EOS
    SAMPLE_SIZE.times do |i|
      file.puts <<-EOS
  #{'%.20f' % sinusoid(i.to_f / (SAMPLE_SIZE-1).to_f)},
      EOS
    end
    file.puts <<-EOS
};
    EOS
    file.puts <<-EOS
const float dsp_func_log_tbl[] = {
    EOS
    SAMPLE_SIZE.times do |i|
      file.puts <<-EOS
  #{'%.20f' % log(i.to_f / (SAMPLE_SIZE-1).to_f)},
      EOS
    end
    file.puts <<-EOS
};
    EOS
    file.puts <<-EOS
const float dsp_func_exp_tbl[] = {
    EOS
    SAMPLE_SIZE.times do |i|
      file.puts <<-EOS
  #{'%.20f' % (1.0 - log(1.0 - (i.to_f / (SAMPLE_SIZE-1).to_f)))},
      EOS
    end
    file.puts <<-EOS
};
    EOS
    file.puts <<-EOS
const float dsp_func_perlin_5order_tbl[] = {
    EOS
    SAMPLE_SIZE.times do |i|
      t = i.to_f / (SAMPLE_SIZE-1).to_f
      file.puts <<-EOS
  #{'%.20f' % (1.0 - (6.0 * (t**5).abs - 15.0 * (t**4) + 10 * (t**3).abs))},
      EOS
    end
    file.puts <<-EOS
};
    EOS
  end
end

if $0 == __FILE__
  create_table_cpp_file

  #values = []
  #SAMPLE_SIZE.times do |x|
  #  t = x.to_f / (SAMPLE_SIZE-1).to_f
  #  y = (1.0 - (6.0 * (t**5).abs - 15.0 * (t**4) + 10 * (t**3).abs))
  #  values << [x, y]
  #end
  #g = Gruff::Line.new
  #g.title = "Curve"
  #g.dataxy 'Data', values
  #g.write('test2.png')
end
