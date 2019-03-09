import cv2

class RegionVisualizer(object):

    def __init__(self,region_data_list):
        self.region_data_list = region_data_list
        self.is_first_image = True
        self.window_name = 'tracking_regions'
        self.window_num_pad = 0,5
        self.window_region_color = (0,255,0)
        self.window_led_color = (0,0,255)
        self.window_object_color = (255,255,0)
        self.window_object_radius = 5

    def update(self, image, tracked_objects):
        if image is None:
            return
        if self.is_first_image: 
            cv2.namedWindow(self.window_name,cv2.WINDOW_NORMAL)
            cv2.moveWindow(self.window_name, 100, 100)
            cv2.resizeWindow(self.window_name, 800,600)
            self.is_first_image = False 

        for num, region_data in enumerate(self.region_data_list):
            x0, y0 = region_data['x0'], region_data['y0']
            x1, y1 = region_data['x1'], region_data['y1']
            x_text = x0+self.window_num_pad[0]
            y_text = y0-self.window_num_pad[1]
            
            cv2.rectangle(image, (x0,y0), (x1,y1), self.window_region_color, 1)
            cv2.putText(image,'{}'.format(num), (x_text,y_text) , cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.window_region_color)

            if region_data.has_key('leds'):
                for led_data in region_data['leds']:
                    led_x0 = led_data['x0']
                    led_y0 = led_data['y0']
                    led_x1 = led_data['x1']
                    led_y1 = led_data['y1']
                    cv2.rectangle(image, (led_x0,led_y0), (led_x1,led_y1), self.window_led_color, 1)
                    
            for obj in tracked_objects:
                center = int(obj.position.x), int(obj.position.y)
                cv2.circle(image, center, self.window_object_radius, self.window_object_color, 1) 

        cv2.imshow(self.window_name,image)
        cv2.waitKey(1)

