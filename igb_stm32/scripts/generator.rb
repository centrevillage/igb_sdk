# vim:set fileencoding=utf-8

require 'rexml/document'
require 'set'
require 'erb'
require 'pathname'

require_relative '_schema'
require_relative '_name_config'

# 覚環境に応じたDeviceInfo 構造体のコードを生成する。
# また、各機能が有効かどうか判定するためのマクロを自動生成する
class SVDParser
  def initialize(filename)
    @filename = filename

    # ペリフェラルをグループごとにざっくり分割
    @parsed = {}
    @periph_to_group = {}
    @all_interrupts = {}
  end

  attr_reader :parsed, :periph_to_group

  def debug(text)
    puts text
  end

  def parse
    open(@filename) do |file|
      @doc = REXML::Document.new(file)
      REXML::XPath.match(@doc, "/device/peripherals/peripheral").each do |peripheral|
#         debug "[peripheral] name = #{peripheral.elements['name'].text}"
        scan_peripheral(peripheral)
      end

      #pp @parsed

      #@cpp_structs.delete_if { |group_name, structs| !structs || structs.empty? }

      #pp @cpp_structs
    end
  end

  # : ペリフェラルの定義から対応するハッシュマップを構築
  def scan_peripheral(peripheral)
    parent_doc_name = peripheral.attribute("derivedFrom")&.value
    # TODO: 毎回親ペリフェラルを検索するのは重いのでキャッシュする
    parent_doc = if parent_doc_name
      REXML::XPath.first(@doc, "/device/peripherals/peripheral/name[text()='#{parent_doc_name}']/parent::peripheral")
    end
    #debug "parent - #{parent_doc.elements['name'].text}" if parent_doc
    peripheral_name = peripheral.elements['name'].text
    group_name = peripheral.elements['groupName']&.text || parent_doc&.elements['groupName']&.text || peripheral_name

    peripheral_name = fix_ambiguous_peripheral_name(peripheral_name)
    group_name = fix_ambiguous_group_name(group_name)

    #debug "[#{group_name}] #{peripheral_name}"

    group_name = group_name.to_sym
    peripheral_name = peripheral_name.to_sym

    @parsed[group_name] ||= {}
    @parsed[group_name][peripheral_name] ||= {}

    @parsed[group_name][peripheral_name] = _scan_peripheral(parent_doc) if parent_doc
    @parsed[group_name][peripheral_name].update(_scan_peripheral(peripheral))
    @all_interrupts.update(@parsed[group_name][peripheral_name][:interrupts] || {})

    @periph_to_group[peripheral_name] = group_name
  end

  def search_interrupts(pattern = /\A.*\Z/)
    @all_interrupts.select {|k, v| k.to_s =~ pattern }
  end

  def _scan_peripheral(peripheral)
    result = {}
    peripheral.elements.each do |elem|
      if elem.text && elem.text.size > 0
        result[elem.name.to_sym] = elem.text
      end
    end

    result[:addressBlock] = scan_address_block(peripheral.elements['addressBlock']) if peripheral.elements['addressBlock']

    result[:registers] = {}
    REXML::XPath.match(peripheral, "registers/register").each do |register|
      reg  = scan_register(register)
      result[:registers][reg[:name].to_sym] = reg
    end

    result[:interrupts] = {}
    REXML::XPath.match(peripheral, "interrupt").each do |interrupt|
      intr = scan_interrupt(interrupt)
      result[:interrupts][intr[:name]] = intr
    end

    result
  end

  # 表記揺れの補正
  def fix_ambiguous_peripheral_name(name)
    case name.to_s
    when 'ADC'
      'ADC1'
    when 'DAC'
      'DAC1'
    when /\ASYSCFG_/
      'SYSCFG'
    else
      name
    end
  end

  def fix_ambiguous_group_name(name)
    case name.to_s
    when 'TIMs'
      'TIM'
    when /\ASYSCFG_/
      'SYSCFG'
    else
      name
    end
  end

  def scan_address_block(address_block)
    Hash[*(
      address_block.elements.flat_map {|elem|
        [elem.name.to_sym, elem.text]
      }
    )]
  end

  def scan_register(register)
    register_attrs = Hash[*(
      register.elements.select{|elem| elem&.text }.flat_map {|elem|
        [elem.name.to_sym, elem.text]
      }
    )]

    fields = []
    REXML::XPath.match(register, "fields/field").each do |field|
      fields << Hash[*(field.elements.flat_map {|elem| [elem.name.to_sym, elem&.text]})]
    end
    register_attrs[:fields] = fields

    return register_attrs
  end

  def scan_interrupt(interrupt)
    Hash[*(
      interrupt.elements.flat_map {|elem|
        [elem.name.to_sym, elem.text]
      }
    )]
  end
end

class DeviceFileParser
  def initialize(filename)
    @filename = filename
    @parsed = {}
    @parsed[:gpio_af] ||= {}
    @af_max_idx = 7
  end

  attr_reader :parsed, :af_max_idx

  def parse
    open(@filename) do |file|
      @doc = REXML::Document.new(file)
      REXML::XPath.match(@doc, "/modm/device/driver/gpio").each do |gpio_pin|
        scan_gpio_pin(gpio_pin)
      end
    end
  end

  # TODO: refactoring
  def scan_gpio_pin(gpio_pin)
    port_str = gpio_pin.attribute('port').to_s
    pin = gpio_pin.attribute('pin').to_s.to_i

    @parsed[:gpio] ||= {}
    parsed_signals = []

    gpio_pin.elements.select {|elem| elem.name == 'signal'}.each do |signal|
      parsed_signal = Hash[*(signal.attributes.flat_map {|name, value| [name.to_sym,  value]})]
      parsed_signals << parsed_signal

      if (af = signal.attribute('af').to_s.to_i) and (driver = signal.attribute('driver').to_s.upcase.to_sym)
        @af_max_idx = af if af > @af_max_idx
        periph_name = "#{driver}#{signal.attribute('instance')}".to_sym
        @parsed[:gpio_af][driver] ||= {}
        @parsed[:gpio_af][driver][periph_name] ||= {}
        @parsed[:gpio_af][driver][periph_name][signal.attribute('name').to_s.to_sym] = {
          af: af,
          port: "GPIO#{port_str.upcase}",
          pin: pin
        }
      end
    end

    @parsed[:gpio][port_str.to_sym] ||= {}
    @parsed[:gpio][port_str.to_sym][pin.to_i] = parsed_signals
  end
end

class CppSrcGenerator
  def initialize(mcu)
    @mcu = mcu
    @cpp_structs = {}
    @periph_names = Set.new
    @af_info_structs = {}
    @bus_names = Set.new
  end

  attr_reader :mcu

  def process
    svd_file_name = Pathname.new(__dir__).join('..', 'svd', "#{NameConfig.mcu_to_svd(mcu)}.svd")
    @svd_parser = SVDParser.new(svd_file_name)
    @svd_parser.parse

    device_file_name = Pathname.new(__dir__).join('..', 'device_file', "#{NameConfig.mcu_to_df(mcu)}.xml")
    @df_parser = DeviceFileParser.new(device_file_name)
    @df_parser.parse

    gen_structs
    update_structs_for_bus_clock

    gen_af_info_structs

    gen_device_header_file
    gen_mcu_header_file
  end

  def gen_device_header_file
    file_symbol = mcu.to_s
    include_guard_name = "IGB_STM32_BASE_MCU_DEVICE_#{file_symbol.upcase}_H"
    header_file_name = "#{file_symbol}.hpp".downcase
    schema = CppStructSchema::SCHEMA
    open(Pathname.new(__dir__).join('..', 'base', 'mcu', 'device', header_file_name), 'w') do |hpp_file|
      open(Pathname.new(__dir__).join('_peripheral_tmpl.hpp.erb')) do |erb_file|
        erb = ERB.new(erb_file.read, trim_mode: "<>")
        hpp_file.print erb.result(binding)
      end
    end
  end

  def gen_mcu_header_file
    mcu_header_name = "#{mcu.to_s.downcase}.hpp"
    device_file_name = "#{mcu.to_s.downcase}.hpp"
    include_guard_name = "IGB_STM32_BASE_MCU_#{mcu.to_s.upcase}_H"
    open(Pathname.new(__dir__).join('..', 'base', 'mcu', mcu_header_name), 'w') do |hpp_file|
      open(Pathname.new(__dir__).join('_mcu_tmpl.hpp.erb')) do |erb_file|
        erb = ERB.new(erb_file.read, trim_mode: "<>")
        hpp_file.print erb.result(binding)
      end
    end
  end

  def gen_structs
    @svd_parser.parsed.each do |group_name, group|
      @cpp_structs[group_name] ||= []

      @svd_parser.parsed[group_name].keys.sort_by {|k|
        if k.to_s =~ /\d+\z/
          sprintf("%04d", k.to_s.gsub(/\D/, '').to_i)
        else
          k.to_s
        end
      }.each do |peripheral_name|
        next if include_black_list?(peripheral_name)
        struct = CppStructSchema.create(group_name)
        struct[:periph] = peripheral_name if struct
        case group_name.to_sym
        when :GPIO
          struct[:attrs][:p_gpio][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
        when :TIM
          struct[:attrs][:p_tim][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
          irqn = fetch_tim_irqn_name(peripheral_name)
          struct[:attrs][:irqn][:value] = irqn
          description = @svd_parser.parsed[group_name][peripheral_name][:description]
          if description =~ /\AAdvanced/
            struct[:attrs][:category][:value] = 'TimCategory::ADVANCED'
          elsif description =~ /\AGeneral/
            struct[:attrs][:category][:value] = 'TimCategory::GENERAL'
          elsif description =~ /\ABasic/
            struct[:attrs][:category][:value] = 'TimCategory::BASIC'
          else
            #struct[:attrs][:type][:value] = 'TIMType::UNKNOWN'
            raise "Unknown Timer [description=#{description}]"
          end
        when :RCC
          struct[:attrs][:p_rcc][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
        when :I2C
          struct[:attrs][:p_i2c][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
        when :SPI
          struct[:attrs][:p_spi][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
        when :USART
          struct[:attrs][:p_usart][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
          irqn = fetch_usart_irqn_name(peripheral_name)
          struct[:attrs][:irqn][:value] = irqn
        when :ADC
          struct[:attrs][:p_adc][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
          irqn = fetch_adc_irqn_name(peripheral_name)
          struct[:attrs][:irqn][:value] = irqn
        when :DAC
          struct[:attrs][:p_dac][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
          irqn = fetch_dac_irqn_name(peripheral_name)
          struct[:attrs][:irqn][:value] = irqn
        when :TSC
          struct[:attrs][:p_tsc][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
          irqn = fetch_tsc_irqn_name(peripheral_name)
          struct[:attrs][:irqn][:value] = irqn
        when :DMA
          struct[:attrs][:p_dma][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
        when :EXTI
          struct[:attrs][:p_exti][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
        when :SYSCFG
          struct[:attrs][:p_syscfg][:value] = peripheral_name
          struct[:attrs][:addr][:value] = "#{peripheral_name}_BASE"
        else
          next
        end
        @periph_names << peripheral_name.to_s
        @cpp_structs[group_name] << struct
      end
    end
  end

  def regulate_irqn_name(name)
    if name =~ /_IRQ\Z/
      "#{name}n"
    elsif name =~ /_IRQn\Z/
      name
    else
      "#{name}_IRQn"
    end
  end

  def fetch_tim_irqn_name(peripheral_name)
    interrupts = @svd_parser.search_interrupts(/(\A|_)#{peripheral_name}(_|\z)/)
    name = if interrupts.size > 1
      names = @svd_parser.search_interrupts(/(\A|_)#{peripheral_name}_UP.*\z/).keys
      if names.empty?
        interrupts.keys.first.to_s
      else
        names.first.to_s
      end
    elsif interrupts.size == 0
      peripheral_name.to_s
    else
      interrupts.keys.first.to_s
    end
    regulate_irqn_name(name)
  end

  def fetch_dma_irqn_name(peripheral_name, channel)
    name = @svd_parser.search_interrupts(/\A#{peripheral_name}(Channel|CH|_)+#{channel}(_|\z)/).keys.first
    return name unless name
    name = name.gsub(/_CH/, '_Channel')
    regulate_irqn_name(name)
  end

  def fetch_adc_irqn_name(peripheral_name)
    adc_number = peripheral_name.to_s.gsub(/\A.*(\d+)/, '\1').to_i
    name = @svd_parser.search_interrupts(/\AADC.*#{adc_number}/).keys.first
    regulate_irqn_name(name)
  end

  def fetch_dac_irqn_name(peripheral_name)
    dac_number = peripheral_name.to_s.gsub(/\A.*(\d+)/, '\1').to_i
    name = 
      @svd_parser.search_interrupts(/DAC.*#{dac_number}/).keys.first ||
      (dac_number == 1 && @svd_parser.search_interrupts(/DAC(\z|_)/).keys.first)
    regulate_irqn_name(name)
  end

  def fetch_usart_irqn_name(peripheral_name)
    number = peripheral_name.to_s.gsub(/\A.*(\d+)/, '\1')&.to_i
    name = @svd_parser.search_interrupts(/US?ART#{number || ''}/).keys.first
    name = name.gsub(/_EXTI\d+/, '')
    regulate_irqn_name(name)
  end

  def fetch_tsc_irqn_name(peripheral_name)
    interrupts = @svd_parser.search_interrupts(/(\A|_)#{peripheral_name}(_|\z)/)
    regulate_irqn_name(interrupts.keys.first.to_s)
  end

  def fetch_exti_irqn_name(peripheral_name, line)
    name = @svd_parser.search_interrupts(/(\A|_)#{peripheral_name}#{line}(_|\z)/).keys.first
    unless name
      # maybe range pattern? (EXTI9_5_IRQ etc)
      range_names = @svd_parser.search_interrupts(/(\A|_)#{peripheral_name}\d+_\d+(_|\z)/).keys
      range_names.each do |range_name|
        # extract range idx
        si, ei = range_name.scan(/(\A|_)#{peripheral_name}(\d+)_(\d+)(\z|_)/).map{|m| [m[1], m[2]]}.first.map(&:to_i)
        si, ei = ei, si if si > ei # swap
        if (si .. ei).include?(line.to_i)
          name = range_name
          break
        end
      end
    end
    return name unless name
    regulate_irqn_name(name)
  end

  def gen_af_info_structs
    @df_parser.parsed[:gpio_af].each do |group_name, group|
      @af_info_structs[group_name] ||= {}
      group.each do |periph_name, periph|
        @af_info_structs[group_name][periph_name] ||= {}
        if struct = CppStructSchema.create_gpio_af(group_name.to_sym)
          periph.each do |func_name, data|
            @af_info_structs[group_name][periph_name][func_name] ||= []
            struct[:attrs][:p_gpio][:value] = data[:port]
            struct[:attrs][:pin_bit][:value] = "1UL << #{data[:pin]}"
            struct[:attrs][:af_idx][:value] = data[:af]
            @af_info_structs[group_name][periph_name][func_name] << struct
          end
        end
      end
    end
  end

  BUSNAME_TO_PERIPHNAME = {
    IOPA: :GPIOA, IOPB: :GPIOB, IOPC: :GPIOC, IOPD: :GPIOD,
    IOPE: :GPIOE, IOPF: :GPIOF, IOPG: :GPIOG, IOPH: :GPIOH,
    IOPI: :GPIOI, IOPJ: :GPIOJ, IOPK: :GPIOK, IOPL: :GPIOL,
    IOPM: :GPIOM, IOPN: :GPION, IOPO: :GPIOO, IOPP: :GPIOP,
  }

  def bus_name_to_periph_name
    @bus_name_to_periph_name ||= {}
    @bus_name_to_periph_name[mcu] ||= BUSNAME_TO_PERIPHNAME.merge(
      case mcu.to_sym
      when :stm32f072xb, :stm32f030x6
        {
          DAC: :DAC1,
          ADC:  :ADC1
        }
      when :stm32h750xx
        {
          DAC12:  [:DAC1, :DAC2], 
          ADC12:  [:ADC1, :ADC2], 
          USART7: :UART7,
          USART8: :UART8,
        }
      when :stm32f303x8, :stm32f303xc, :stm32f303xe
        {
          ADC12:  [:ADC1, :ADC2], 
          ADC34:  [:ADC3, :ADC4], 
          USART4: :UART4,
          USART5: :UART5,
          DAC: :DAC1,
          DMA: :DMA1,
        }
      when :stm32g431xx
        {
          ADC12:  [:ADC1, :ADC2], 
          ADC345:  [:ADC3, :ADC4, :ADC5], 
        }
      when :stm32g031xx
        {
          ADC: :ADC1,
          DMA: :DMA1,
        }
      when :stm32f446xx
        {
          DAC: :DAC1,
        }
      else
        {}
      end
    )
    @bus_name_to_periph_name[mcu]
  end

  def include_black_list?(periph_name)
    case mcu.to_sym
    when :stm32f030x6
      %W(SPI2 I2C2 TIM6 TIM15 USART2).include?(periph_name.to_s)
    when :stm32f072xb
      false
    when :stm32h750xx
      %W(ADC12_Common ADC3_Common).include?(periph_name.to_s)
    when :stm32f303x8
      %W(I2S2ext I2S3ext SPI2 SPI3 I2C2 I2C3).include?(periph_name.to_s)
    when :stm32f303xc
      %W(I2S2ext I2S3ext SPI2 SPI3 SPI4 I2C2 I2C3 GPIOG GPIOH TIM20 ADC1_2 ADC3_4).include?(periph_name.to_s)
    when :stm32f446xx
      %W(C_ADC).include?(periph_name.to_s)
    when :stm32g031xx
      %W(LPUART).include?(periph_name.to_s)
    else
      false
    end
  end

  def update_structs_for_bus_clock
    # AHBxLP系はRSTRレジスタがないなど特殊なのでひとまず対象外
    %i(
    AHB
    APB
    APB1
    AHB1
    AHB2
    AHB3
    AHB4
    APB1H
    APB1L
    APB2
    APB3
    APB4
    IOP
    ).each do |bus_name|
      registers = @svd_parser.parsed[:RCC][:RCC][:registers]
      enrs = [registers[:"#{bus_name}ENR"], registers[:"#{bus_name}ENR1"], registers[:"#{bus_name}ENR2"]]
      enrs.each_with_index do |enr, idx|
        next unless enr

        _bus_name = bus_name.dup
        if idx > 0
          _bus_name = "#{bus_name}_#{idx}"
        end
        @bus_names << _bus_name
        enr[:fields].each do |field|
          bus_periph_name = field[:name][0..-3].to_sym
          #puts "_bus_name = #{_bus_name}, bus_periph_name = #{bus_periph_name}"
          if periph_name = bus_name_to_periph_name[bus_periph_name] || bus_periph_name
            [*periph_name].each do |pname|
              if group_name = @svd_parser.periph_to_group[pname]
                if periph_struct = @cpp_structs[group_name].find {|elem| elem[:periph].to_sym == pname}
                  periph_struct[:bus] = {
                    name: _bus_name,
                    bit_offset: field[:bitOffset].to_i,
                  }
                end
              end
            end
          end
        end
      end
    end
  end

  def process_all
    NameConfig.available_mcu.each do |mcu|
      g = CppSrcGenerator.new(mcu)
      g.process(mcu)
    end
  end
end

def get_periheral_names
  peripheral_names = Set.new
  
  # fetch group name (= peripheral name)
  Dir["#{__dir__}/../svd/*.svd"].each do |svd_file_name|
  #Dir["#{__dir__}/svd/STM32F072x.svd"].each do |svd_file_name|
    open(svd_file_name) do |file|
      doc = REXML::Document.new(file)
      peripheral_names += REXML::XPath.match(doc, "/device/peripherals/peripheral").map {|peripheral|
        peripheral.elements['groupName']&.text # || peripheral.elements['name']&.text
      }.compact
    end
  end
  peripheral_names
end

if __FILE__ == $0
  #peripheral_names = get_periheral_names
  #puts "[peripheral names] #{peripheral_names}"

  generator = CppSrcGenerator.new(:stm32g031xx)
  generator.process
end
