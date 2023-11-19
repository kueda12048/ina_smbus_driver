import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '../'))
import ina228_smbus

bus_number = 1 # Raspberry pi 03, 05 pin
# bus_number = 7 # Jetson orin nano 03, 05 pin
adress = 0x40
#  https://strawberry-linux.com/catalog/items?code=11228
shunt = 0.002 # Ohm
max_current = 70 # A

ina = ina228_smbus.INA228_SMBus(bus_number, adress)
ina.setShunt(shunt, max_current)

print('mode: ', hex(ina.mode))

# set the ADC config
ina.averaging_count = ina228_smbus.AveragingCount.COUNT_16
print('averaging_count: ', ina228_smbus.AveragingCount.get_averaging_count(ina.averaging_count))

# measure the values
print('temp: %.2f degC' % ina.temperature_degc)
print('voltage: %.3f V' % ina.voltage_V)
print('current: %.3f A' % ina.current_A)
print('power: %.4f W' % ina.power_W)
print('energy: %.4f J' % ina.energy_J)
print('charge: %.4f C' % ina.charge_C)
