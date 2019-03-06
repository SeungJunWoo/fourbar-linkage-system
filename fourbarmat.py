import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

######################### you can change these values ########################
base = 200  # link length setting
crank = 100
coupler = 150
follower = 100
offset1 = 50  # trajectory point setting: horizontal offset from coupler link
offset2 = 30  # trajectory point setting: vertical offset from coupler link
x = [200.0, 250.0]  # crank fixed point position
##############################################################################

# value initialize
w = [x[0]+base, x[1]]
y = [0.0, 0.0]
z = [0.0, 0.0]
t = [0.0, 0.0]
c = 0
th1 = 0.0
th2 = 0.0
trajectoryx = []
trajectoryy = []
x1=[]
x2=[]
x3=[]
x4=[]
x5=[]
x6=[]
y1=[]
y2=[]
y3=[]
y4=[]
y5=[]
y6=[]

# initial pose finding process
possible = False
# possible pose finding by rotating crank and follower links.
for i in range(361):
    y[0] = (x[0]+(crank*np.cos((45+i)*(math.pi/180.0))))
    y[1] = (x[1]+(crank*np.sin((45+i)*(math.pi/180.0))))
    for j in range(361):
        z[0] = (w[0]+(follower *
                      np.cos((45+j)*(math.pi/180.0))))
        z[1] = (w[1]+(follower *
                      np.sin((45+j)*(math.pi/180.0))))
        distance = math.sqrt(((z[0]-y[0])**2)  # distance between crank and
                             + ((z[1]-y[1])**2))  # followers moving points.
        if abs(distance-coupler) < 2:  # if error is smaller than 2 pixel,
            th1 = 45+i  # crank initial angle
            th2 = 45+j  # follower initial angle
            possible = True  # your link length setting is possible
            break

# control loop
if possible == False:  # if pose finding failed
    print "impossible link lengths!"
if possible == True:
    for i in range(0,1440):
        #crank's moving point position update
        y[0]=(x[0]+(crank*np.cos((th1+i/4.0)*(math.pi/180.0))))
        y[1]=(x[1]+(crank*np.sin((th1+i/4.0)*(math.pi/180.0))))
        
        #follower angle finding process
        movepossible=False
        for k in range(40):
            z[0]=(w[0]+(follower*np.cos((th2-5+k/4.0)*(math.pi/180.0))))
            z[1]=(w[1]+(follower*np.sin((th2-5+k/4.0)*(math.pi/180.0))))
            distance=math.sqrt(((z[0]-y[0])**2)
                                +((z[1]-y[1])**2))
            if abs(distance-coupler)<2:
                th2=th2-5+k/4
                movepossible=True
                break

        #coupler link's angle finding 
        dx=z[0]-y[0]
        dy=z[1]-y[1]
        c_angle=math.degrees(math.atan2(dy,dx)) #coupler's angle

        #trajectory point's position
        t[0]=(y[0]+(offset2*np.cos((c_angle+90)*(math.pi/180)))
                                     +(offset1*np.cos((c_angle)*(math.pi/180))))
        t[1]=(y[1]+(offset2*np.sin((c_angle+90)*(math.pi/180)))
                                     +(offset1*np.sin((c_angle)*(math.pi/180))))
        #orthogonal point's position(between trajectory point and coupler) 
        o=[0.0,0.0]
        o[0]=(y[0]+(offset1*np.cos((c_angle)*(math.pi/180))))
        o[1]=(y[1]+(offset1*np.sin((c_angle)*(math.pi/180))))
            
        x1.append(x[0])
        x2.append(y[0])
        x3.append(z[0])
        x4.append(w[0])
        x5.append(t[0])
        x6.append(o[0])
        y1.append(x[1])
        y2.append(y[1])
        y3.append(z[1])
        y4.append(w[1])
        y5.append(t[1])
        y6.append(o[1])
        if movepossible==False:
            th1=th1+i/4.0
            break

    for i in range(1,1440):
        #crank's moving point position update
        y[0]=(x[0]+(crank*np.cos((th1-i/4.0)*(math.pi/180.0))))
        y[1]=(x[1]+(crank*np.sin((th1-i/4.0)*(math.pi/180.0))))
        
        #follower angle finding process
        movepossible=False
        for k in range(40):
            z[0]=(w[0]+(follower*np.cos((th2-5+k/4.0)*(math.pi/180.0))))
            z[1]=(w[1]+(follower*np.sin((th2-5+k/4.0)*(math.pi/180.0))))
            distance=math.sqrt(((z[0]-y[0])**2)
                                +((z[1]-y[1])**2))
            if abs(distance-coupler)<2:
                th2=th2-5+k/4
                movepossible=True
                break

        #coupler link's angle finding 
        dx=z[0]-y[0]
        dy=z[1]-y[1]
        c_angle=math.degrees(math.atan2(dy,dx)) #coupler's angle

        #trajectory point's position
        t[0]=(y[0]+(offset2*np.cos((c_angle+90)*(math.pi/180)))
                                     +(offset1*np.cos((c_angle)*(math.pi/180))))
        t[1]=(y[1]+(offset2*np.sin((c_angle+90)*(math.pi/180)))
                                     +(offset1*np.sin((c_angle)*(math.pi/180))))
        #orthogonal point's position(between trajectory point and coupler) 
        o=[0.0,0.0]
        o[0]=(y[0]+(offset1*np.cos((c_angle)*(math.pi/180))))
        o[1]=(y[1]+(offset1*np.sin((c_angle)*(math.pi/180))))
            
        x1.append(x[0])
        x2.append(y[0])
        x3.append(z[0])
        x4.append(w[0])
        x5.append(t[0])
        x6.append(o[0])
        y1.append(x[1])
        y2.append(y[1])
        y3.append(z[1])
        y4.append(w[1])
        y5.append(t[1])
        y6.append(o[1])
        if movepossible==False:
            break

# # create a time array from 0..100 sampled at 0.05 second steps
dt = 0.05

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, 600), ylim=(0, 600))
ax.grid()

line1, = ax.plot([], [], 'o-', lw=2)
line2, = ax.plot([], [], 'o-', lw=2)
line3, = ax.plot([], [], 'o-', lw=2)
line4, = ax.plot([], [], 'o-', lw=2)
line5, = ax.plot([], [], 'o-', lw=2)
line6, = ax.plot([], [], 'o-', lw=2)

def init():
    line1.set_data([], [])
    line2.set_data([], [])
    line3.set_data([], [])
    line4.set_data([], [])
    line5.set_data([], [])
    line6.set_data([], [])

    return line1,line2,line3,line4,line5,line6


def animate(i):
    line1x = [x1[i], x2[i]]
    line1y = [y1[i], y2[i]]
    line2x = [x2[i], x3[i]]
    line2y = [y2[i], y3[i]]
    line3x = [x3[i], x4[i]]
    line3y = [y3[i], y4[i]]
    line4x = [x4[i], x1[i]]
    line4y = [y4[i], y1[i]]
    line5x = [x5[i], x6[i]]
    line5y = [y5[i], y6[i]]
    line6x = []
    line6y = []
    for j in range(i):
        line6x.append(x5[j])
        line6y.append(y5[j])

    line1.set_data(line1x, line1y)
    line2.set_data(line2x, line2y)
    line3.set_data(line3x, line3y)
    line4.set_data(line4x, line4y)
    line5.set_data(line5x, line5y)
    line6.set_data(line6x, line6y)

    return line1,line2,line3,line4,line5,line6


ani = animation.FuncAnimation(fig, animate, np.arange(1, len(x1)),
                              interval=2, blit=True, init_func=init, repeat=False)

# ani.save('double_pendulum.mp4', fps=15)
plt.show()
