from led_pwm_tlc5947_proxy import LedPwmTLC5947Proxy
import rospy

class LedScheduler(object):

    def __init__(self,region_index,led_index,params) :

        # Extract parameters 
        self.region_index = region_index
        self.led_index = led_index
        self.params = dict(params)
        self.dev_number = 0

        # State
        self.led_on = False 
        self.last_on_t  = 0.0
        self.activation_count = 0

        self.led_proxy= LedPwmTLC5947Proxy()
        self.turn_off_led()

    def __del__(self):
        self.turn_off_led()
    
    def turn_on_led(self,t):
        if not self.led_on:
            self.led_proxy.set(self.dev_number,self.params['chan'],self.params['on_value'])
            self.activation_count += 1
            self.led_on = True
            self.last_on_t = t

    def turn_off_led(self):
        if self.led_on:
            self.led_proxy.set(self.dev_number,self.params['chan'], self.params['off_value'])
            self.led_on = False

    def update(self, t, inside_region): 
        if self.led_on:
            if (t - self.last_on_t) > self.params['on_duration']:
                self.turn_off_led()
                #print('region {} led {} off'.format(self.region_index, self.led_index))
        else:
            if inside_region:
                if (t - self.last_on_t) > (self.params['on_duration'] + self.params['refractory_duration']):
                    self.turn_on_led(t)   
                    #print('region {} led {} on'.format(self.region_index, self.led_index))





