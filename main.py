from functions import *

O = np.array([60.0, 30.0, 140.0]) # angle
endPos = np.array([50.0, 20.0, -55.0]) # x,y,z
startPos = FK(O)
# print(np.abs(startPos - endPos) > endPos-0.001)
final_O = jacobianIK(O, endPos, startPos, 0.001)
print("Final orientation:", final_O)