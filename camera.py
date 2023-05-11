from picamera2 import Picamera2, Preview
import time
import glob

#Definer camera og opløsning
cam = Picamera2()
x = 800
cam_res = (int(x*(3/4)), x)
config = cam.create_still_configuration(main={"size": cam_res}, lores={"size": cam_res}, display="main")
cam.configure(config)

#Tænd kameraet
cam.start()
time.sleep(0.5)

#Main loop. Indtil bliver slukket:
while 1:
    #Lav en liste over alle filer der i mappen "images/", som har fil typen .jpg
    list = glob.glob("images/*.jpg")
    #Hvis der er færre end 12 billeder
    if len(list) < 12:
        #Tag 12 billeder
        for i in range(12):
                cam.capture_file("images/a%d.jpg" % i)
                list.append("images/a%d.jpg" % i)
                time.sleep(1/24)
    