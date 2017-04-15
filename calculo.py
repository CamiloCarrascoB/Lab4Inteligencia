import math

w=0.404
rel=1.818*10**(-4)  ##r/N
thetaA=math.pi/2

nl=900
nr=200

Sl=2*math.pi*nl*rel
Sr=2*math.pi*nr*rel

deltaS=(Sl+Sr)/2
deltaA=(Sr-Sl)/w

#CASOS DONDE deltaA=0
#deltax=deltaS*math.cos(thetaA*math.pi/180)
#deltay=deltaS*math.sin(thetaA*math.pi/180)

x1=-(deltaS*math.sqrt(2*(1-math.cos(deltaA))))/(deltaA)
x2=math.sin(thetaA-math.asin(math.sin(deltaA))/(math.sqrt(2*(1-math.cos(deltaA)))))

y1=(deltaS*math.sqrt(2*(1-math.cos(deltaA))))/(deltaA)
y2=math.cos(thetaA-math.asin(math.sin(deltaA))/(math.sqrt(2*(1-math.cos(deltaA)))))


deltax=x1*x2
deltay=y1*y2

print 'Delta x= ' + str(deltax)
print 'Delta y= '+ str(deltay)
print 'Delta theta= ' + str(deltaA*180/math.pi)
