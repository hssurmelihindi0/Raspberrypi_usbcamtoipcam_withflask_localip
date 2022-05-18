from flask import Flask, render_template, Response, request
import cv2
import datetime, time
import os, sys
import numpy as np
from threading import Thread
import RPi.GPIO as GPIO
from time import sleep


global capture,rec_frame, grey, switch, neg, face, rec, out,slider
slider=0
capture=0
grey=0
neg=0
face=0
switch=1
rec=0

servo_pin=6
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin,GPIO.OUT)
pwm=GPIO.PWM(servo_pin,50)
pwm.start(1)


#make shots directory to save pics
try:
    os.mkdir('./shots')
except OSError as error:
    pass

#Load pretrained face detection model    
'''net = cv2.dnn.readNetFromCaffe('./saved_model/deploy.prototxt.txt', './saved_model/res10_300x300_ssd_iter_140000.caffemodel')
'''
#instatiate flask app  
app = Flask(__name__, template_folder='/home/pi/Desktop/app')


camera = cv2.VideoCapture(1)

def record(out):
    global rec_frame
    while(rec):
        time.sleep(0.05)
        out.write(rec_frame)
    
def gen_frames():  # generate frame by frame from camera
    global out, capture,rec_frame
    while True:
        success, frame = camera.read() 
        if success:
            
            if(grey):
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if(neg):
                frame=cv2.bitwise_not(frame)    
            if(capture):
                capture=0
                now = datetime.datetime.now()
                p = os.path.sep.join(['shots', "shot_{}.png".format(str(now).replace(":",''))])
                cv2.imwrite(p, frame)
            
            if(rec):
                rec_frame=frame
                frame= cv2.putText(cv2.flip(frame,1),"Recording...", (0,25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),4)
                frame=cv2.flip(frame,1)
            
                
            try:
                ret, buffer = cv2.imencode('.jpg', cv2.flip(frame,1))
                frame = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            except Exception as e:
                pass
                
        else:
            pass

#render temlate html code            
@app.route('/')
def index():
    return render_template('index.html')
#servo motor control request control
@app.route("/test",methods=["POST"])
def slider():
	slider =request.form["slider"]
	pwm.ChangeDutyCycle(float(slider))
	sleep(5)
	pwm.ChangeDutyCycle(0)
	return render_template('index.html')  
#get video from camera    
@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')
#controlled requests
@app.route('/requests',methods=['POST','GET'])

def tasks():
    global switch,camera
    if request.method == 'POST':
        if request.form.get('click') == 'Capture': ##take a photo
            global capture
            capture=1
        elif  request.form.get('grey') == 'Grey': ##turn grey camera vision
            global grey
            grey=not grey
        elif  request.form.get('neg') == 'Negative': ##turn negative camera vision
            global neg
            neg=not neg  
        elif  request.form.get('stop') == 'Stop/Start': #start and stop camera
            
            if(switch==1):
                switch=0
                camera.release()
                cv2.destroyAllWindows()
                
            else:
                camera = cv2.VideoCapture(0)
                switch=1
        elif  request.form.get('rec') == 'Start/Stop Recording': ##start recording
            global rec, out
            rec= not rec
            if(rec):
                now=datetime.datetime.now() 
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                out = cv2.VideoWriter('vid_{}.avi'.format(str(now).replace(":",'')), fourcc, 20.0, (640, 480))
                #Start new thread for recording the video
                thread = Thread(target = record, args=[out,])
                thread.start()
            elif(rec==False):
                out.release()
                          
                 
    elif request.method=='GET':
        return render_template('index.html')
    return render_template('index.html')
    
    
  


if __name__ == '__main__':
    app.run(debug=True,host='192.168.1.25',port=5000,threaded=True)
