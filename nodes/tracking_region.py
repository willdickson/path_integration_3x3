from rect_region import RectRegion
from led_region import LedRegion
from operator import attrgetter

class TrackingRegion(RectRegion):

    def __init__(self, index, params):
        super(TrackingRegion,self).__init__(index, params)
        self.led_region_list = []
        for i, led_base_param in enumerate(self.params['leds']):
            led_param = dict(params['stimulation'])
            led_param.update(led_base_param)
            self.led_region_list.append(LedRegion(i,led_param))

    def update(self,t,tracked_objects): 
        contained_objects = [obj for obj in tracked_objects if self.contains(obj)]
        if contained_objects:
            #print('region {} has object'.format(self.index))
            max_object = max(contained_objects, key=attrgetter('size'))
            for led_region in self.led_region_list:
                led_region.update(t, max_object)
        else:
            #print('region {} empty'.format(self.index))
            pass






        




