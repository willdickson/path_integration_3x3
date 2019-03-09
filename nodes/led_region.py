from rect_region import RectRegion
from led_scheduler import LedScheduler

class LedRegion(RectRegion):

    def __init__(self, index, params):
        super(LedRegion,self).__init__(index,params)
        self.led_scheduler = LedScheduler(index,params)

    def update(self,t, obj):
        #if self.contains(obj):
        #    print('  led {}  contains obj'.format(self.index))
        self.led_scheduler.update(t, self.contains(obj))

