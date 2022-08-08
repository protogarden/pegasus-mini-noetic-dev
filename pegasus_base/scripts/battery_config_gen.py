
#!/usr/bin/env python
import sys
import time
sys.path.append("current_sensors_scripts")
from DFRobot_INA219 import INA219
from std_msgs.msg import Float32
import yaml

#ina.reset()   
#ina2.reset()  



class SocPlot():
    def __init__(self):
        ina219_reading_mA = 1000
        ext_meter_reading_mA = 1000

        self.ina = INA219(1, INA219.INA219_I2C_ADDRESS3)
                 

        #begin return True if succeed, otherwise return False
        while not self.ina.begin():
            time.sleep(2)




        self.ina.set_bus_RNG(self.ina.bus_vol_range_32V)
        self.ina.set_PGA(self.ina.PGA_bits_8)
        self.ina.set_bus_ADC(self.ina.adc_bits_12, self.ina.adc_sample_8)
        self.ina.set_shunt_ADC(self.ina.adc_bits_12, self.ina.adc_sample_8)
        self.ina.set_mode(self.ina.shunt_and_bus_vol_con)
        time.sleep(1)
        self.ina.linear_cal(ina219_reading_mA, ext_meter_reading_mA)

        #ina.reset()                                     #Resets all registers to default values

        self.max_bat_voltage = 12.6
        self.min_bat_voltage = 10.6
        self.bat_range = self.max_bat_voltage - self.min_bat_voltage
        self.battery_resistance = 0.1

        


    def run(self):
        self.last_time = time.time()
        self.battery_measure_voltage = self.ina.get_bus_voltage_V()
        battery_current = self.ina.get_current_mA()/1000
        self.last_actual_voltage = self.battery_measure_voltage - battery_current*self.battery_resistance
        self.accum_watt_time = 0
        self.accum_amp_hour = 0
        self.accum_voltage = 0
        voltage_interval = 0.1
        battery_voltage = []
        accum_power = []
        percentage = []
        self.original_batt_ah = 5.2
        count = 0
        self.last_actual_average_voltage = 12.6
        self.average_actual_voltage = 0
        self.test_voltage = 11

        while self.last_actual_average_voltage > self.min_bat_voltage:

            ###ina219 measures from IN- to GND for bus and is currently wired so a + currents means it is charging hence the positive of battery going into IN- to comform to standards hence battery voltage
            # is equal to bus. If wired the otherway the voltage drop across shunt voltage need to be accounted for : self.battery_voltage = ina.get_bus_voltage_V() + (ina.get_shunt_voltage_mV()/1000)
            self.battery_measure_voltage = self.ina.get_bus_voltage_V() 
            battery_current = self.ina.get_current_mA()/1000
            self.battery_actual_voltage = self.battery_measure_voltage - battery_current*self.battery_resistance
            power = self.battery_actual_voltage*battery_current
            self.time_now = time.time()
            d_time = self.time_now - self.last_time
            power_interval = (power*d_time)/3600
            current_interval = (battery_current*d_time)/3600
            self.accum_watt_time += power_interval
            self.accum_amp_hour += current_interval
            self.accum_voltage += self.battery_actual_voltage

            self.last_time = self.time_now


            count += 1
            if count == 100:
                self.average_actual_voltage = self.accum_voltage/100
                count = 0
                print("average", self.average_actual_voltage)
                self.accum_voltage = 0

                if self.last_actual_average_voltage - self.average_actual_voltage > voltage_interval:
                    battery_voltage.append(self.average_actual_voltage)
                    accum_power.append(self.accum_watt_time)
                    self.last_actual_average_voltage = self.average_actual_voltage
                    print("current battery array", battery_voltage)
                    print("current accum array", accum_power)

                
                
                



        for i in  range(len(battery_voltage)):
            percent = (self.accum_watt_time - accum_power[i])/self.accum_watt_time
            percentage.append(percent)

        SOH = - self.accum_amp_hour / self.original_batt_ah
        print(accum_power)
        dict_file = [{'Battery_Voltage': battery_voltage},{'Battery Percentage': percentage},{'Accumulative Dissipated Power Wh': accum_power},{'State of Health': [SOH]}]
        with open(r'../config/battery_config.yaml', 'w') as file:
            documents = yaml.dump(dict_file, file)
            
            
            


if __name__ == '__main__':
    
    soc_plot = SocPlot()
    soc_plot.run()
    
    

     