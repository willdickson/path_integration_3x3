import rospy
from rect_region import RectRegion
from led_region import LedRegion
from operator import attrgetter

class TrackingRegion(RectRegion):

    def __init__(self, region_index, params):
        super(TrackingRegion,self).__init__(params)
        self.index = region_index
        self.led_region_list = []
        for led_index, led_base_param in enumerate(self.params['leds']):
            led_param = dict(params['stimulation'])
            led_param.update(led_base_param)
            self.led_region_list.append(LedRegion(region_index,led_index,led_param))

    def update(self,t,tracked_objects): 
        contained_objects = [obj for obj in tracked_objects if self.contains(obj)]
        region_data = {}
        if contained_objects:
            max_object = max(contained_objects, key=attrgetter('size'))
            region_data['object'] = max_object
            region_data['index'] = self.index
            region_data['leds'] = []
            for led_region in self.led_region_list:
                led_data = led_region.update(t, max_object)
                region_data['leds'].append(led_data)
        return region_data





        




