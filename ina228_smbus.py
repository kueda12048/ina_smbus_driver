# SPDX-License-Identifier: MIT

"""
`ina228_smbus`

Python driver for the TI INA228 (current/voltage/power monitoring).
It has been implemented with reference to the following.
https://github.com/adafruit/Adafruit_CircuitPython_INA260/blob/main/adafruit_ina260.py
"""

from smbus import SMBus
from typing import NoReturn


_REG_CONFIG         = 0x00  # Configuration (R/W)
_REG_ADC_CONFIG     = 0x01  # ADC Configuration (R/W)
_REG_SHUNT_CAL      = 0x02  # Shunt Calibration (R/W)
_REG_SHUNT_TEMPCO   = 0x03  # Shunt Temperature Coefficient  (R)
_REG_VSHUNT         = 0x04  # Shunt Voltage Measurement (R)
_REG_VBUS           = 0x05  # Bus Voltage Measurement (R)
_REG_DIETEMP        = 0x06  # Temperature Measurement (R)
_REG_CURRENT        = 0x07  # SHUNT VOLTAGE REGISTER (R)
_REG_POWER          = 0x08  # POWEER REGISTER (R)
_REG_ENERGY         = 0x09  # Energy Result (R)
_REG_CHARGE         = 0x0A  # Charge Result (R)
_REG_DIAG_ALRT      = 0x0B  # Diagnostic Flags and Alert (R/W)
_REG_SOVL           = 0x0C  # Shunt Overvoltage Threshold (R/W)
_REG_SUVL           = 0x0D  # Shunt Undervoltage Threshold (R/W)
_REG_BOVL           = 0x0E  # Bus Overvoltage Threshold (R/W)
_REG_BUVL           = 0x0F  # Bus Undervoltage Threshold (R/W)
_REG_TEMP_LIMIT     = 0x10  # Bus Overvoltage Threshold (R/W)
_REG_PWR_LIMIT      = 0x11  # Bus Undervoltage Threshold (R/W)
_REG_MANUFACTURER_ID= 0x3E  # Manufacturer ID (R)
_REG_DEVICE_ID      = 0x3F  # Device ID (R)

class Mode:
    """Modes avaible to be set

    +--------------------+---------------------------------------------------------------------+
    | Mode               | Description                                                         |
    +====================+=====================================================================+
    | ``Mode.CONTINUOUS``| Default: The sensor will continuously measure the bus voltage and   |
    |                    | shunt voltage across the shunt resistor to calculate ``power`` and  |
    |                    | ``current``                                                         |
    +--------------------+---------------------------------------------------------------------+
    | ``Mode.TRIGGERED`` | The sensor will immediately begin measuring and calculating current,|
    |                    | bus voltage, and power. Re-set this mode to initiate another        |
    |                    | measurement                                                         |
    +--------------------+---------------------------------------------------------------------+
    | ``Mode.SHUTDOWN``  |  Shutdown the sensor, reducing the quiescent current and turning off|
    |                    |  current into the device inputs. Set another mode to re-enable      |
    +--------------------+---------------------------------------------------------------------+

    """

    SHUTDOWN = 0x0
    TRIGGERED = 0x7
    CONTINUOUS = 0xF

class ConversionTime:
    """Options for ``current_conversion_time`` or ``voltage_conversion_time``

    +----------------------------------+------------------+
    | ``ConversionTime``               | Time             |
    +==================================+==================+
    | ``ConversionTime.TIME_50_us``    | 50 us            |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_84_us``    | 84 us            |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_150_us``   | 150 us           |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_280_us``   | 280 us           |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_540_us``   | 540 us           |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_1_052_ms`` | 1.052 ms (Default) |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_2_074_ms`` | 2.074 ms         |
    +----------------------------------+------------------+
    | ``ConversionTime.TIME_4_120_ms`` | 4.120 ms         |
    +----------------------------------+------------------+

    """

    TIME_50_us      = 0x0
    TIME_84_us      = 0x1
    TIME_150_us     = 0x2
    TIME_280_us     = 0x3
    TIME_540_us     = 0x4
    TIME_1_052_ms   = 0x5
    TIME_2_074_ms   = 0x6
    TIME_4_120_ms   = 0x7

    @staticmethod
    def get_seconds(time_enum: int) -> float:
        """Retrieve the time in seconds giving value read from register"""
        conv_dict = {
            0: 50e-6,
            1: 84e-6,
            2: 150e-6,
            3: 280e-6,
            4: 540e-6,
            5: 1.052e-3,
            6: 2.074e-3,
            7: 4.120e-3,
        }
        return conv_dict[time_enum]

class AveragingCount:
    """Options for ``averaging_count``

    +-------------------------------+------------------------------------+
    | ``AveragingCount``            | Number of measurements to average  |
    +===============================+====================================+
    | ``AveragingCount.COUNT_1``    | 1 (Default)                        |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_4``    | 4                                  |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_16``   | 16                                 |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_64``   | 64                                 |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_128``  | 128                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_256``  | 256                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_512``  | 512                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_1024`` | 1024                               |
    +-------------------------------+------------------------------------+

    """

    COUNT_1     = 0x0
    COUNT_4     = 0x1
    COUNT_16    = 0x2
    COUNT_64    = 0x3
    COUNT_128   = 0x4
    COUNT_256   = 0x5
    COUNT_512   = 0x6
    COUNT_1024  = 0x7

    @staticmethod
    def get_averaging_count(avg_count: int) -> float:
        """Retrieve the number of measurements giving value read from register"""
        conv_dict = {0: 1, 1: 4, 2: 16, 3: 64, 4: 128, 5: 256, 6: 512, 7: 1024}
        return conv_dict[avg_count]
    
class RWBits:
    """
    :param int num_bits: The number of bits in the field.
    :param int register_address: The register address to read the bit from
    :param int lowest_bit: The lowest bits index within the byte at ``register_address``
    :param int register_width: The number of bytes in the register.
    :param bool signed: Handling negative values.
    """
    def __init__(self, num_bits: int, register_address: int, lowest_bit: int, register_width: int, signed: bool = False) -> None:
        self.bit_mask = ( (1<<num_bits) -1 ) << lowest_bit
        if self.bit_mask >= 1 << (register_width * 8):
            raise ValueError("Cannot have more bits than register size")
        self.num_bits = num_bits
        self.register_address = register_address
        self.register_width = register_width
        self.lowest_bit = lowest_bit
        self.signed = signed

    def __get__(self, obj, objtype) -> int:
        read_data = obj.i2c.read_i2c_block_data(obj.address, self.register_address, self.register_width)
        val = 0
        for i, data in enumerate(read_data):
            val += data << (self.register_width-1-i)*8
        val = (val & self.bit_mask) >> self.lowest_bit

        if not self.signed:
            return val
        if val & (1 << (self.num_bits-1) ): # true : negative
            # print(val)
            # print(((1<<self.num_bits)-1))
            val = -( ( ~val&((1<<self.num_bits)-1) ) + 1 )
        return val
    
    def __set__(self, obj, value: int) -> None:
        if not isinstance(value, int):
            raise TypeError("value must be an integer")
        read_data = obj.i2c.read_i2c_block_data(obj.address, self.register_address, self.register_width)
        # print(read_data)
        val = 0
        for i, data in enumerate(read_data):
            val += data << (self.register_width-1-i)*8
        val &= (~self.bit_mask & ( (1<<self.register_width*8) -1 ) ) # Set the target bit to 0
        val |= (value << self.lowest_bit)
        send_data = []
        for i in range(self.register_width):
            send_data.append( (val>>( (self.register_width-1-i)*8 )) & 0xFF )
        # print(send_data)
        obj.i2c.write_i2c_block_data(obj.address, self.register_address, send_data)

class ROBits(RWBits):
    def __set__(self, obj, value: int) -> NoReturn:
        raise AttributeError()

class INA228_SMBus:

    def __init__(self, bus_number: int, address: int = 0x40) -> None:
        self.address = address
        self.i2c = SMBus(bus_number)

        if self._manufacturer_id != self.TEXAS_INSTRUMENT_ID:
            raise RuntimeError(
                "Failed to find Texas Instrument ID, read "
                + f"{self._manufacturer_id} while expected {self.TEXAS_INSTRUMENT_ID}"
                " - check your wiring!"
            )
        
        if self._device_id != self.INA228_ID:
            raise RuntimeError(
                f"Failed to find INA228 ID, read {self._device_id} while expected {self.INA228_ID}"
                " - check your wiring!"
            )

        print('manufacturer_id: ', hex(self._manufacturer_id))
        print('device_id: ', hex(self._device_id))

    
    TEXAS_INSTRUMENT_ID = 0x5449
    INA228_ID = 0x228


    reset_bit = RWBits(1, _REG_CONFIG, 15, 2)
    """Setting this bit to 1 generates a system reset. Reset all registers to default values."""
    reset_acc = RWBits(1, _REG_CONFIG, 14, 2)
    """Setting this bit to 1 resets the contents of accumulation registers ENERGY and CHARGE to 0"""
    convdly = RWBits(8, _REG_CONFIG, 6, 2)
    """Sets the Delay for initial ADC conversion in steps of 2 ms. 0h=0s, 1h=2ms, FFh=510ms"""
    tempcomp = RWBits(1, _REG_CONFIG, 5, 2)
    """Enables temperature compensation of an external shunt. 0h=Disabled, 1h=Enabled"""
    adcrange = RWBits(1, _REG_CONFIG, 4, 2)
    """Shunt full scale range selection across IN+ and IN-. 0h=±163.84 mV, 1h=±40.96 mV"""


    mode = RWBits(4, _REG_ADC_CONFIG, 12, 2)
    """The mode that the INA268 is operating in. Must be one of
    ``Mode.CONTINUOUS``, ``Mode.TRIGGERED``, or ``Mode.SHUTDOWN``
    """
    voltage_conversion_time = RWBits(3, _REG_ADC_CONFIG, 9, 2)
    """The conversion time taken for the bus voltage measurement"""
    current_conversion_time = RWBits(3, _REG_ADC_CONFIG, 6, 2)
    """The conversion time taken for the current measurement"""
    temperature_conversion_time = RWBits(3, _REG_ADC_CONFIG, 3, 2)
    """The conversion time taken for the temperature measurement"""
    averaging_count = RWBits(3, _REG_ADC_CONFIG, 0, 2)
    """The window size of the rolling average used in continuous mode"""


    shunt_cal = RWBits(15, _REG_SHUNT_CAL, 0, 2)
    """The register provides the device with a conversion constant value
    that represents shunt resistance used to calculate current value inAmperes"""


    tempco = RWBits(14, _REG_SHUNT_TEMPCO, 0, 2)
    """Temperature coefficient of the shunt for temperature compensation correction. 
    Calculated with respect to +25 °C. The full scale value of the register is 16383 ppm/°C.
    The 16 bit register provides a resolution of 1ppm/°C/LSB
    0h = 0 ppm/°C
    3FFFh = 16383 ppm/°C"""


    vshunt = ROBits(20, _REG_VSHUNT, 4, 3)
    """Differential voltage measured across the shunt output. Two's complement value.
    Conversion factor:
    312.5 nV/LSB when ADCRANGE = 0
    78.125 nV/LSB when ADCRANGE = 1"""


    vbus = ROBits(20, _REG_VBUS, 4, 3, False)
    """Bus voltage output. Two's complement value, however always positive.
    Conversion factor: 195.3125 µV/LSB"""
    
    
    dietemp = ROBits(16, _REG_DIETEMP, 0, 2, True)
    """Internal die temperature measurement. Two's complement value.
    Conversion factor: 7.8125 m°C/LSB"""


    current = ROBits(20, _REG_CURRENT, 4, 3, True)
    """Calculated current output in Amperes. Two's complement value."""


    power = ROBits(24, _REG_POWER, 0, 3)
    """Calculated power output. Output value in watts. Unsigned representation. Positive value."""


    energy = ROBits(40, _REG_ENERGY, 0, 5)
    """Calculated energy output. Output value is in Joules.Unsigned representation. Positive value."""


    charge = ROBits(40, _REG_CHARGE, 0, 5, True)
    """Calculated charge output. Output value is in Coulombs.Two's complement value."""


    alatch = RWBits(1, _REG_DIAG_ALRT, 15, 2)
    """
    0h = Transparent mode: the Alert pin and Flag bit reset to the idle state when the fault has been cleared.
    1h = Latched mode: the Alert pin and Alert Flag bit remain active following a fault until the DIAG_ALRT Register has been read
    """
    cnvr = RWBits(1, _REG_DIAG_ALRT, 14, 2)
    """
    0h = Disable conversion ready flag on ALERT pin
    1h = Enables conversion ready flag on ALERT pin
    """
    slowalert = RWBits(1, _REG_DIAG_ALRT, 15, 2)
    """
    0h = ALERT comparison on non-averaged (ADC) value
    1h = ALERT comparison on averaged value
    """


    sovl = RWBits(16, _REG_SOVL, 0, 2)
    """Sets the threshold for comparison of the value to detect Shunt Overvoltage (overcurrent protection)."""

    suvl = RWBits(16, _REG_SUVL, 0, 2)
    """Sets the threshold for comparison of the value to detect Shunt Undervoltage (undercurrent protection)."""
    


    _manufacturer_id = ROBits(16, _REG_MANUFACTURER_ID, 0, 2)
    """Manufacturer identification bits"""
    _device_id = ROBits(12, _REG_DEVICE_ID, 4, 2)
    """Device identification bits"""
    _revision_id = ROBits(4, _REG_DEVICE_ID, 0, 2)
    """Device revision identification bits"""

    @property
    def voltage_V(self) -> float:
        return self.vbus * 195.3125e-6
    
    @property
    def current_A(self) -> float:
        """The current (between V+ and V-) in mA"""
        # if self.mode == Mode.TRIGGERED:
        #     while self._conversion_ready_flag == 0:
        #         pass
        #return self._raw_current * 1.25
        return self.current*self.current_LSB
        #return self.current

    @property
    def temperature_degc(self) -> float:
        return self.dietemp * 7.8125e-3
    
    @property
    def power_W(self) -> float:
        return self.power * 3.2 * self.current_LSB
    
    @property
    def energy_J(self) -> float:
        return self.energy * 16 * 3.2 * self.current_LSB
    
    @property
    def charge_C(self) -> float:
        return self.charge * self.current_LSB


    def setShunt(self, Rshunt: float, Maximum_expected_current: float):
        """
        set the shunt_cal and current_LSB

        :param float Rshunt: shunt resistor [Ω].
        :param float Maximum_expected_current: Max current [A]
        """
        self.Maximum_expected_current = Maximum_expected_current
        self.current_LSB = Maximum_expected_current / 2**19
        # print(self.current_LSB)
        shunt_cal_val = 13107.2e6 * self.current_LSB * Rshunt
        if self.adcrange == 1:
            shunt_cal_val *= 4
        # print(shunt_cal_val)
        self.shunt_cal = int(shunt_cal_val)
