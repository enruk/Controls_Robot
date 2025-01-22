from math import sqrt, atan2, degrees, radians, cos, sin

def pick_angle(q1_1,q1_2,q2_1,q2_2,upper_limit,lower_limit,L_A1,L_B1,D):
    # decision for q2
    X_q1_1 = L_A1 * cos(radians(q1_1))
    X_q1_2 = L_A1 * cos(radians(q1_2))
    X_q2_1 = L_B1 * cos(radians(q2_1)) + D
    X_q2_2 = L_B1 * cos(radians(q2_2)) + D
    
    
    q_soll = [0,0]
    
    
    diff = []
    diff.append(X_q2_1 - X_q1_1)
    diff.append(X_q2_1 - X_q1_2)
    diff.append(X_q2_2 - X_q1_1)
    diff.append(X_q2_2 - X_q1_2)
    
    
    max_diff = max(diff)
    max_diff_index = diff.index(max_diff)
    print("Max Diff Index: " + str(max_diff_index))
    print("Max Diff: " + str(max_diff))
    
    
    q1_valid = check_angle_is_valid(q1_1,upper_limit,lower_limit) or check_angle_is_valid(q1_2,upper_limit,lower_limit)
    q2_valid = check_angle_is_valid(q2_1,upper_limit,lower_limit) or check_angle_is_valid(q2_2,upper_limit,lower_limit)
    
    
    q1_1_valid = check_angle_is_valid(q1_1,upper_limit,lower_limit)
    q1_2_valid = check_angle_is_valid(q1_2,upper_limit,lower_limit)
    q2_1_valid = check_angle_is_valid(q2_1,upper_limit,lower_limit)
    q2_2_valid = check_angle_is_valid(q2_2,upper_limit,lower_limit)
    

    error_Msg_PosNotReachable = "At least one joint is out of range"
    error_Msg_JointsTooClose = "Joints are to close to each other, risk of mechanical damage"

    case = 0
    
    
    if q1_valid and q2_valid:
        
        if q1_1_valid and not q1_2_valid and q2_1_valid and not q2_2_valid: 
            case = 1
            if diff[0] > 0:
                q_soll[0] = q1_1
                q_soll[1] = q2_1
            else:
                print(error_Msg_JointsTooClose)
                
        elif q1_1_valid and not q1_2_valid and not q2_1_valid and q2_2_valid:
            case = 2
            if diff[2] > 0:
                q_soll[0] = q1_1
                q_soll[1] = q2_2
            else:
                print(error_Msg_JointsTooClose)
                
        elif not q1_1_valid and q1_2_valid and q2_1_valid and not q2_2_valid:
            case = 3
            if diff[1] > 0:
                q_soll[0] = q1_2
                q_soll[1] = q2_1
            else:
                print(error_Msg_JointsTooClose)
                
        elif not q1_1_valid and q1_2_valid and not q2_1_valid and q2_2_valid:
            case = 4
            if diff[3] > 0:
                q_soll[0] = q1_2
                q_soll[1] = q2_2
            else:
                print(error_Msg_JointsTooClose)
                
        elif q1_1_valid and q1_2_valid and not q2_1_valid and q2_2_valid:
            case = 5
            if diff[3] > diff[2] and diff[3] > 0:
                q_soll[0] = q1_2
                q_soll[1] = q2_2
            elif diff[2] >= diff[3] and diff[2] > 0:
                q_soll[0] = q1_1
                q_soll[1] = q2_2
            else:
                print(error_Msg_JointsTooClose)
                
        elif q1_1_valid and q1_2_valid and q2_1_valid and not q2_2_valid:
            case = 6
            if diff[1] > diff[0] and diff[1] > 0:
                q_soll[0] = q1_2
                q_soll[1] = q2_1
            elif diff[0] >= diff[1] and diff[0] > 0:
                q_soll[0] = q1_1
                q_soll[1] = q2_1
            else:
                print(error_Msg_JointsTooClose)
                
        elif q1_1_valid and not q1_2_valid and q2_1_valid and q2_2_valid:
            case = 7
            if diff[2] > diff[0] and diff[2] > 0:
                q_soll[0] = q1_1
                q_soll[1] = q2_2
            elif diff[0] >= diff[2] and diff[0] > 0:
                q_soll[0] = q1_1
                q_soll[1] = q2_1
            else:
                print(error_Msg_JointsTooClose)
                
        elif not q1_1_valid and q1_2_valid and q2_1_valid and q2_2_valid:
            case = 8
            if diff[3] > diff[1] and diff[3] > 0:
                q_soll[0] = q1_2
                q_soll[1] = q2_2
            elif diff[1] >= diff[3] and diff[1] > 0:
                q_soll[0] = q1_2
                q_soll[1] = q2_1
            else:
                print(error_Msg_JointsTooClose)
                
        elif q1_1_valid and q1_2_valid and q2_1_valid and q2_2_valid:
            case = 9
            if max_diff_index == 0:
                q_soll[0] = q1_1
                q_soll[1] = q2_1
            elif max_diff_index == 1:
                q_soll[0] = q1_2
                q_soll[1] = q2_1
            elif max_diff_index == 2:
                q_soll[0] = q1_1
                q_soll[1] = q2_2
            elif max_diff_index == 3:
                q_soll[0] = q1_2
                q_soll[1] = q2_2
    else:
        print(error_Msg_PosNotReachable)
    
    print("Case: "+ str(case))
    return q_soll

def check_angle_is_valid(angle,upper_limit,lower_limit):
    valid = False
    if angle > lower_limit and angle < upper_limit:
        valid = True
    return valid 

def ikp(L_A1, L_A2, L_B1, L_B2, D):
# Axis q1
    a1 = 2 * L_A1 * Y
    b1 = -2 * L_A1 * X
    c1 = X**2 + Y**2 + L_A1**2 - L_A2**2
    sum_1 = a1**2 + b1**2 - c1**2
    d1_plus = sqrt(sum_1)
    d1_minus = -sqrt(sum_1)
        
    q1_1 = degrees(atan2(c1, d1_plus) + atan2(b1,a1))
    q1_2 = degrees(atan2(c1, d1_minus) + atan2(b1,a1))



    # Axis q2
    a2 = 2 * L_B1 * Y
    b2 = 2 * L_B1 * (D - X)
    c2 = X**2 + Y**2 + D**2 + L_B1**2 - L_B2**2 - 2*D*X
    sum_2 = a2**2 + b2**2 - c2**2
    d2_plus = sqrt(sum_2)
    d2_minus = -sqrt(sum_2)
        
    q2_1 = degrees(atan2(c2, d2_plus) + atan2(b2,a2))
    q2_2 = degrees(atan2(c2, d2_minus) + atan2(b2,a2))

    return q1_1,q1_2,q2_1,q2_2


# init
X = 0
Y = 200

D = 100
L_A1 = 200
L_A2 = 200
L_B1 = 200
L_B2 = 200
lower_limit = -20
upper_limit = 200
q = [2]

q1_1_grad,q1_2_grad,q2_1_grad,q2_2_grad = ikp(L_A1,L_A2,L_B1,L_B2,D)

# temp output
print(q1_1_grad)
print(q1_2_grad)
print(q2_1_grad)
print(q2_2_grad)


q_soll_grad = pick_angle(q1_1_grad,q1_2_grad,q2_1_grad,q2_2_grad,upper_limit,lower_limit,L_A1,L_B1,D)


print(q_soll_grad[0])
print(q_soll_grad[1])
    
    
    
        


