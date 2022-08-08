
#!/usr/bin/env python
import sys
import time
sys.path.append("current_sensors_scripts")
from DFRobot_INA219 import INA219
import rospy
from sensor_msgs.msg import BatteryState
ina219_reading_mA = 1000
ext_meter_reading_mA = 1000
from std_msgs.msg import Float32
import numpy as np ##install sudo apt-get install python-numpy python-scipy python-matplotlib
from scipy.interpolate import interp1d
import pylab
import yaml #pip3 install pyyaml


ina = INA219(1, INA219.INA219_I2C_ADDRESS3)
ina2 = INA219(1, INA219.INA219_I2C_ADDRESS4)            


                #Change I2C address by dialing DIP switch

#begin return True if succeed, otherwise return False
while not ina.begin():
    time.sleep(2)


while not ina2.begin():
    time.sleep(2)




#ina.reset()   
#ina2.reset()  

ina.set_bus_RNG(ina.bus_vol_range_32V)
ina.set_PGA(ina.PGA_bits_8)
ina.set_bus_ADC(ina.adc_bits_12, ina.adc_sample_8)
ina.set_shunt_ADC(ina.adc_bits_12, ina.adc_sample_8)
ina.set_mode(ina.shunt_and_bus_vol_con)
time.sleep(1)
ina2.set_bus_RNG(ina.bus_vol_range_32V)
ina2.set_PGA(ina.PGA_bits_8)
ina2.set_bus_ADC(ina.adc_bits_12, ina.adc_sample_8)
ina2.set_shunt_ADC(ina.adc_bits_12, ina.adc_sample_8)
ina2.set_mode(ina.shunt_and_bus_vol_con)
time.sleep(1)
ina.linear_cal(ina219_reading_mA, ext_meter_reading_mA)
ina2.linear_cal(ina219_reading_mA, ext_meter_reading_mA)  

class PowerMonitoring():
    def __init__(self):
        ina219_reading_mA = 1000
        ext_meter_reading_mA = 1000

        ina = INA219(1, INA219.INA219_I2C_ADDRESS3)
        ina2 = INA219(1, INA219.INA219_I2C_ADDRESS4)            

        #begin return True if succeed, otherwise return False
        while not ina.begin():
            time.sleep(2)


        while not ina2.begin():
            time.sleep(2)

        ina.set_bus_RNG(ina.bus_vol_range_32V)
        ina.set_PGA(ina.PGA_bits_8)
        ina.set_bus_ADC(ina.adc_bits_12, ina.adc_sample_8)
        ina.set_shunt_ADC(ina.adc_bits_12, ina.adc_sample_8)
        ina.set_mode(ina.shunt_and_bus_vol_con)
        time.sleep(1)
        ina2.set_bus_RNG(ina.bus_vol_range_32V)
        ina2.set_PGA(ina.PGA_bits_8)
        ina2.set_bus_ADC(ina.adc_bits_12, ina.adc_sample_8)
        ina2.set_shunt_ADC(ina.adc_bits_12, ina.adc_sample_8)
        ina2.set_mode(ina.shunt_and_bus_vol_con)
        time.sleep(1)
        ina.linear_cal(ina219_reading_mA, ext_meter_reading_mA)
        ina2.linear_cal(ina219_reading_mA, ext_meter_reading_mA)  
        #ina.reset()                                     #Resets all registers to default values

        self.max_bat_voltage = 12.6
        self.min_bat_voltage = 10.5
        self.bat_range = self.max_bat_voltage - self.min_bat_voltage


        with open("../config/battery_config.yaml") as f:
            pubsec = yaml.safe_load(f)

        y =  pubsec[0]['Battery_Voltage']
        b_percent = pubsec[1]['Battery Percentage']
        x = []

        for i in range(len(y)):

            z = 1 - b_percent[i]
            x.append(z)
        



        #f_nearest = interp1d(x, y, kind='nearest')
        #f_linear  = interp1d(x, y)
        self.f_cubic= interp1d(y, x, kind='cubic')


    def run(self):
        rospy.init_node('power_interface_node', anonymous=True)
        battery_pub = rospy.Publisher('/battery_status', BatteryState, queue_size=5)
        charge_current_pub = rospy.Publisher('/charge_current', Float32, queue_size=5)
        msg = BatteryState()
        rate = rospy.Rate(1)
        self.battery_average_interval = 10
        self.battery_last_time = rospy.Time.now()
        accum_time = 0
        count = 0
        accum_voltage = 0
        self.average_voltage = ina.get_bus_voltage_V()

        while not rospy.is_shutdown():
           
            
            '''
            print("Battery")
            print ("Shunt Voltage   : %.2f mV" % ina.get_shunt_voltage_mV())
            print ("Bus Voltage     : %.3f V" % ina.get_bus_voltage_V())
            print ("Current         : %.f mA" % ina.get_current_mA())
            print ("Power           : %.f mW" % ina.get_power_mW())
            self.battery_voltage = ina.get_bus_voltage_V() + (ina.get_shunt_voltage_mV()/1000)
            print ("Battery Voltage: %.3f V" % self.battery_voltage)
            print (" ")

            print("Charging")
            print ("Shunt Voltage   : %.2f mV" % ina2.get_shunt_voltage_mV())
            print ("Bus Voltage     : %.3f V" % ina2.get_bus_voltage_V())
            print ("Current         : %.f mA" % ina2.get_current_mA())
            print ("Power           : %.f mW" % ina2.get_power_mW())
            charge_voltage = ina2.get_bus_voltage_V() + ina2.get_shunt_voltage_mV()/1000
            print ("Battery Voltage: %.3f V" % charge_voltage)
            print (" ")
            '''
            ###ina219 measures from IN- to GND for bus and is currently wired so a + currents means it is charging hence the positive of battery going into IN- to comform to standards hence battery voltage
            # is equal to bus. If wired the otherway the voltage drop across shunt voltage need to be accounted for : self.battery_voltage = ina.get_bus_voltage_V() + (ina.get_shunt_voltage_mV()/1000)
            self.battery_voltage = ina.get_bus_voltage_V() 
            msg.voltage = self.battery_voltage
            self.battery_average_time = rospy.Time.now()
            d_time = (self.battery_average_time - self.battery_last_time).to_sec()
            self.battery_last_time = self.battery_average_time
            accum_voltage += self.battery_voltage
            accum_time += d_time
     
            count += 1
            if accum_time > 10:
                self.average_voltage = accum_voltage / count
                accum_time = 0
                count = 0
                accum_voltage = 0
              

            if self.average_voltage > self.max_bat_voltage: 
                self.percent_charge_level = 1.0
            elif self.average_voltage < self.min_bat_voltage:
                self.percent_charge_level = 0.0
            else:
                self.percent_charge_level = (1 - self.f_cubic(self.average_voltage))
                self.percent_charge_level = (round(self.percent_charge_level, 2))

            msg.percentage = self.percent_charge_level
            msg.current = ina.get_current_mA()/1000
            battery_pub.publish(msg)

            self.charge_current = ina2.get_current_mA()/1000
            charge_current_pub.publish(self.charge_current)
            rate.sleep()

if __name__ == '__main__':
    
    power_interface = PowerMonitoring()

    try:
        
        power_interface.run()

    except rospy.ROSInterruptException:
        pass
    
    power_interface.stop()
