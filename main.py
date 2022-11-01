import sensor, image, time, lcd, utime
from board import board_info
from motor import Motor
from pid import PID
import KPU as kpu
from fpioa_manager import fm
from Maix import GPIO
import gc

board_info.load({
    'PWM_L': 9,
    'PWM_R': 10,
    'LDIR1': 32,
    'LDIR2': 33,
    'RDIR1': 34,
    'RDIR2': 35,
})

ROIS = [ #[ROI,line,weight]
        (0,200,60,5),
        (0,200,80,5),
        (0,200,100,4),
        (0,200,110,4),
        (0,200,120,3),
        (0,200,130,3),
       ]


labels = ['5', '7', '2', '4', '1', '3', '6', '8']
anchors = [3.53, 2.81, 1.72, 2.19, 3.06, 2.34, 2.41, 2.28, 2.53, 1.5]

#!!!Flag!!!
#!!!Flag!!!
#!!!Flag!!!
flag = 0
cross_flag = 0
init_num = 0
norm_speed = 50
slow_speed = 40
set_speed = norm_speed

task = kpu.load(0x300000)
kpu.init_yolo2(task, 0.5, 0.4, 5, anchors)

motor = Motor()

dir_pid = PID(p=0.15, i=0, d=0)

cam_offset = -60

thresholds = (0, 100, 17, 127, -33, 127)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((224, 224))
sensor.set_hmirror(1)
sensor.set_vflip(1)

lcd.init(type=1)
clock = time.clock() # Tracks FPS.


def get_max_blob(blobs):
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

def follow_line(speed, img, stride=8):
    #巡线算法
    global cross_flag, cam_offset
    centroid_sum = 0
    is_exist = 0
    for r in ROIS:
        lefborder = r[0]
        rigborder = r[1]
        for l in range(r[0],r[1],stride):
            pixel = img.get_pixel(r[2],l)
            lab = image.rgb_to_lab(pixel)
            if lab[0] >= thresholds[0] and lab[0] <= thresholds[1] and lab[1] >= thresholds[2] and lab[1] <= thresholds[3] and lab[2] >= thresholds[4] and lab[2] <= thresholds[5]:
                img.set_pixel(r[2],l,(0,0,255))
                if lefborder == 0:
                    lefborder = l
            elif lefborder != 0:
                rigborder = l
                is_exist = 1
                break
        if lefborder != r[0] and rigborder >= lefborder + 55:
            cross_flag = 1;

        centroid_sum += (lefborder + rigborder - r[0] - r[1] + cam_offset) * r[3] # r[4] is the roi weight.
    #偏差值
    if is_exist:
        d_error = centroid_sum/50
        output=dir_pid.get_pid(d_error,1)
        lspeed = speed+output
        rspeed = speed-output
        motor.run(lspeed, rspeed)
        img.draw_string(0,0, ("%2.1f" %(d_error)),scale=2)
        print(lspeed, rspeed)
    else:
        motor.stop()

def find_nums(img):
    max_id = '0'
    max_value = 0.0
    pos = (0,0,0,0)
    objects = kpu.run_yolo2(task, img)
    if objects:
         for obj in objects:
             pos = obj.rect()
             img.draw_rectangle(pos)
             img.draw_string(pos[0], pos[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 0, 0))
             img.draw_string(0, 200, "x=%d y=%d" %(pos[0], pos[1]), scale=2, color=(255, 0, 0))
             if  obj.value() > max_value:
                max_value = obj.value()
                max_id = labels[obj.classid()]
    return int(max_id), pos

def turn_pos(pos):
    if pos[1] < 80:
        motor.run(35,75)
    elif pos[1] + pos[3] > 80:
        motor.run(75,35)
    utime.sleep_ms(600)


#### boot key ####
fm.register(16, fm.fpioa.GPIOHS0)
key = GPIO(GPIO.GPIOHS0, GPIO.PULL_UP)
while (key.value()==1 or init_num == 0):
    img = sensor.snapshot()
    init_num, _ = find_nums(img)
    lcd.display(img)

delay_time = 0
while(True):
    clock.tick()

    img = sensor.snapshot()
    follow_line(set_speed, img)
    if cross_flag:
        if not flag:
            set_speed = slow_speed
            num, pos = find_nums(img)
            if init_num == num:
                flag = 1
        elif delay_time < 60:
            delay_time += 1
            if delay_time == 60:
                turn_pos(pos)
                set_speed = norm_speed


    img.draw_string(200,0, ("%2d" %(clock.fps())),scale=2)
    lcd.display(img)
    gc.collect()
















