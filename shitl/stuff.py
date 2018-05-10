import time 
head = "ALTITUDE_BAROMETER,ASCENT_RATE,LAS_STATE.v,LAS_STATE.effort,LAS_STATE.status,LAS_STATE.action,VALVE_QUEUE,BALLAST_QUEUE"
timestr = time.strftime("%Y%m%d-%H%M%S")
print(timestr)
f= open("test-outputs/SHTIL-" + timestr + ".csv","w+")
f.write(head+"\n")
f.write(", ".join(str(i) for i in [0.1, 10.1, 12321.01]) + "\n")
f.close()
