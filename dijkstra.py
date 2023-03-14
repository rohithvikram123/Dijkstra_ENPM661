import numpy as np
import matplotlib.pyplot as plt
import cv2 as cv
import heapq as hq
from queue import PriorityQueue

boundry = []
Pth = {}
queue = PriorityQueue()
b_track = []
CheckedList = []

def obstacle_space(space):
    h,w,_ = space.shape
    for l in range(h):
        for m in range(w):
            if ((250-l) - 3 < 0) or ((m) - 3 < 0) or ((250-l) - 247 > 0) or ((m) - 597 > 0): #boundary
                space[l][m] = [0,0,255]
                boundry.append((m,250-l))
            if (m > 95) and (m < 155) and (250-l < 105) and (250-l >2):
                space[l][m] = [0,255,0]
                boundry.append((m,250-l))
            if (m > 100) and (m < 151) and (250-l < 101) and (250-l > 2):   #rectangle1
                space[l][m] = [0,0,255]
                boundry.append((m,250-l))
            if(m > 95) and (m < 155) and (250-l >145) and (250-l < 248):
                space[l][m] = [0,255,0]
                boundry.append((m,250-l))
            if (m > 100) and (m < 151) and (250-l >150) and (250-l < 248):  #rectangle2
                space[l][m] = [0,0,255]
                boundry.append((m,250-l))
            if (((0.577*m)+(250-l)-217.432)>=0) and ((m-230.048)>=0) and (((-0.577*m) + (250-l)-32.567)<=0) and (((0.577*m)+(250-l)- 378.979)<=0) and ((m-369.951)<=0) and (((-0.577*m)+(250-l)+128.980)>=0):
                space[l][m] = [0,255,0]
                boundry.append((m,250-l))
            if (((9375*m)+(16238*(250-l))-3624400)>=0) and (((125*m)-29381)>=0) and (((9375*m)-(16238*(250-l))+435100)>=0) and (((37500*m)+(64951*(250-l))-24240200)<=0) and (((1000*m)-364951)<=0) and (((37500*m)-(64951*(250-l))-8002450)<=0):   #hexagon
                space[l][m] = [0,0,255]
                boundry.append((m,250-l))
            if ((m-455)>=0) and (((2*m)+(250-l)-1156.18)<=0) and (((2*m)-(250-l)-906.18)<=0) and ((250-l)>20) and ((250-l)<230):
                space[l][m] = [0,255,0]
                boundry.append((m,250-l))
            if (((m-460)>=0)) and(((2*m)+(250-l)-1145)<=0)and (((2*m)-(250-l)-895)<=0): #triangle
                space[l][m] = [0,0,255]
                boundry.append((m,250-l))
    return boundry




def User_Inputs_Start(Obs_Coords):
    while True:
        x = int(input("Enter the Initial x node: "))
        y = int(input("Enter the Initial y node: "))
        
        if((x>=0) or (x<=600)) and (y>=0) or (y>250):
            if (x,y) not in Obs_Coords:
                start_node = (x,y)
                return start_node
            else:
                print("The Entered Start Node is in obstacle space")
def User_Inputs_Goal(Obs_Coords):
    while True:
        x = int(input("Enter the Goal x node: "))
        y = int(input("Enter the Goal y node: "))
        #goal_node = (x,y)
        if((x>=0) or (x<=600)) and (y>=0) or (y<=250):
            if (x,y) not in Obs_Coords:
                goal_node=(x,y)
                break
            else:
                print("The Entered Goal Node is in obstacle space")
    return goal_node

def Up_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0],pos[1]+1)
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 

def Down_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0],pos[1]-1)
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 

def Left_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0]-1,pos[1])
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 

def Right_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0]+1,pos[1])
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 

def UpLeft_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0]-1,pos[1]+1)
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1.4
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 

def UpRight_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0]+1,pos[1]+1)
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1.4
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 

def DownLeft_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0]-1,pos[1]-1)
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1.4
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 

def DownRight_function(a,CheckedList,Obs_Coords):
    pos = a[1]
    newPos = (pos[0]+1,pos[1]-1)
    if (newPos not in CheckedList) and (newPos not in Obs_Coords):
        Cost = a[0] + 1.4
        for m in range(queue.qsize()):
            if queue.queue[m][1] == newPos:
                if queue.queue[m][0] > Cost:
                    queue.queue[m] = (Cost,newPos)
                    Pth[newPos] = pos
                    return
                else:
                    return
        queue.put((Cost,newPos))
        Pth[newPos] = pos 


def B_tracking(Pth, initial_pt, goal_pt):
    b_track = []
    K = Pth.get(goal_pt)
    b_track.append(goal_pt)
    b_track.append(K)
    while (K != initial_pt):  
        K = Pth.get(K)
        b_track.append(K)
    b_track.reverse()
    return (b_track)


            
space = np.ones((250,600,3),dtype='uint8')  #Creating an matrix with ones, of the shape of boundry shape

Obs_Coords= obstacle_space(space)           #Creating the obstacle boundries

# for val in Obs_Coords:
#     if val == (101,70):
#         print("the val is present in obstacle", val)

initial_pt = User_Inputs_Start(Obs_Coords)
goal_pt = User_Inputs_Goal(Obs_Coords)
start = (0,initial_pt)
queue.put(start)
while True:
    a = queue.get()
    CheckedList.append(a[1])
    (x,y) = a[1]
    if a[1] != goal_pt:
        if (y+1 < 250):
            Up_function(a,CheckedList,Obs_Coords)
        if ((a[1][1]-1) > 0):
            Down_function(a,CheckedList,Obs_Coords)
        if ((a[1][0]-1) > 0):
            Left_function(a,CheckedList,Obs_Coords)
        if ((a[1][0]+1)<600):
            Right_function(a,CheckedList,Obs_Coords)
        if((a[1][0]-1)>0) and ((a[1][1]+1)<250):
            UpLeft_function(a,CheckedList,Obs_Coords)
        if((a[1][0]+1)<600) and ((a[1][1]+1)<250):
            UpRight_function(a,CheckedList,Obs_Coords)
        if((a[1][0]-1)>0) and ((a[1][1]-1)>0):
            DownLeft_function(a,CheckedList,Obs_Coords)
        if((a[1][0]+1)<600) and ((a[1][1]-1)>0):
            DownRight_function(a,CheckedList,Obs_Coords)    
    else:
        print("success")
        break
b = B_tracking(Pth, initial_pt, goal_pt)
print("path")
print(b)

for i in CheckedList:
    space[250-i[1]][i[0]] = [255,0,0]
    cv.imwrite("output.jpg", space)
    
for j in b:
    space[250-j[1]][j[0]] = [0,255,0]
    

    


# print(initial_pt)
# print(goal_pt)
# print(boundries)
# print(shape)

cv.imshow("obstacle space",space)
cv.waitKey(0)
cv.destroyAllWindows()
# import cv2 as cv
# import numpy as np

# # initialize variables
# space = np.zeros((500,500,3), dtype=np.uint8)
# space.fill(255)

# # define the path and obstacles
# # ...

# # create a video writer
# fourcc = cv.VideoWriter_fourcc(*'mp4v')
# out = cv.VideoWriter('output.mp4', fourcc, 30, (500, 500))

# # loop over each step of the path and write each frame to the video
# for i in CheckedList:
#     space[250-i[1]][i[0]] = [255,0,0]
#     out.write(space)

# for j in b:
#     space[250-j[1]][j[0]] = [0,255,0]
#     out.write(space)

# # release the video writer and destroy any remaining windows
# out.release()
# cv.destroyAllWindows()
