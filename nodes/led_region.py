from rect_region import RectRegion
from led_scheduler import LedScheduler

class LedRegion(RectRegion):

    def __init__(self, region_index, led_index, params):
        super(LedRegion,self).__init__(params)
        self.region_index = region_index
        self.led_index = led_index
        self.led_scheduler = LedScheduler(region_index,led_index,params)

    def update(self,t, obj):
        contains_object = self.contains(obj)
        self.led_scheduler.update(t, contains_object)
        data = {}
        data['contains_object'] = contains_object
        data['activation_count'] = self.led_scheduler.activation_count
        data['index' ] = self.led_index
        return data


