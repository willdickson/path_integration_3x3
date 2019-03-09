
class RectRegion(object):

    def __init__(self, index, params):
        self.index = index
        self.params = dict(params)

    def contains(self, obj):
        x,y = obj.position.x, obj.position.y
        rsp = True 
        if x < self.params['x0'] or x > self.params['x1']:
            rsp = False
        if y < self.params['y0'] or y > self.params['y1']:
            rsp = False
        return rsp
