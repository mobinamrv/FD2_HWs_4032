#Mobina marvdashtipour
#40112341106122

import numpy as np
import math 

def exercise_1():
  

  # Define parameters and constants
  V = PSIDOT = float(input("(PSIDOT)= "))  # Velocity
  PHI = float(input("(PHI=) "))  # Bank angle (degrees)
  PHI_rad=math.radians(PHI) # Bank angle in radians
  G=float(9.81) # Acceleration due to gravity
  TETA1=0.0 # Pitch angle
  PSI=0.0 #YAW_ANGLE

  # Calculate body frame angular rates
  P_body=-V*math.sin(TETA1)
  Q_body=V*math.cos(TETA1)*math.sin(PHI_rad)
  R_body=V*math.cos(TETA1)*math.cos(PHI_rad)

  # Create body frame angular velocity vector
  Angular_velocity_in_body_frame=np.array([[P_body],[Q_body],[R_body]])

  # Display body frame angular velocity

  print('Angular_velocity_in_body_frame= ')
  print(Angular_velocity_in_body_frame)

  # Define inertial to body transformation matrix

  C_inersia_to_body=np.array([[np.cos(PSI)*np.cos(TETA1),np.cos(TETA1)*np.sin(PSI),-np.sin(TETA1)],
  [np.cos(PSI)*np.sin(PHI)*np.sin(TETA1)-np.cos(PHI)*np.sin(PSI),np.cos(PHI)*np.cos(PSI)+np.sin(PHI)*np.sin(TETA1)*np.sin(PSI),
  np.cos(TETA1)*np.sin(PHI)],
  [np.sin(PHI)*np.sin(PSI)+np.cos(PHI)*np.cos(PSI)*np.sin(TETA1),np.cos(PHI)*np.sin(TETA1)*np.sin(PSI)-np.cos(PSI)*np.sin(PHI),
  np.cos(PHI)*np.cos(TETA1)]])

  C_body_to_inertia=C_inersia_to_body.T

  # Calculate inertial frame angular velocity vector

  Angular_velocity_in_inertial_frame=np.dot(C_body_to_inertia, Angular_velocity_in_body_frame)

  # Display inertial frame Angular velocity

  print('Angular_velocity_in_inertial_frame= ')
  print(Angular_velocity_in_inertial_frame)
  #.....................................................

def exercise_2():


  #Input
  Roll_rate = float(input("(Roll rate): "))
  Pitch_rate = float(input("(Pitch rate): "))
  Yaw_rate = float(input("(Yaw rate): "))
  Psidot = float(input("Psidot= "))

  # Constructing the angular velocity vector
  Angular_velocity  = np.array ([[Roll_rate],[Pitch_rate],[Yaw_rate]])

  # Displaying the angular velocity vector
  print("Angular velocity vector")
  print(Angular_velocity)

  # Calculating Euler angels (Theta , Phi , psi )
  Theta = math.asin(-Roll_rate / Psidot) 
  Phi = math.asin(Pitch_rate / (Psidot * math.cos(Theta)))
  Psi = -Roll_rate / math.sin(Theta)

   # Displaying the calculated Euler angles
  print("Calculated Euler angles: ")

  print("Theta (Pitch angle): ")
  print(Theta)

  print("Phi (Roll angle): ")
  print(Phi)

  print("Psi (Yaw angle): ")
  print(Psi)



def exercise_3():

  # Transformation Matrix (Inertia to body)
  C_inersia_to_body = np.array([ [float(input(" (1,1)[0,0]: ")), float(input("(1,2)[0,1]: ")), float(input("(1,3)[0,2]: "))],
  [float(input("(2,1)[1,0]: ")), float(input(" (2,2)[1,1]: ")),float(input(" (2,3)[1,2]: "))],
  [float(input("(3,1)[2,0]: ")), float(input("(3,2)[2,1]: ")),float(input("(3,3)[2,2]: "))]])

  # Check if the matrix is a rotation matrix( determinant should be 1)
  det_C_inersia_to_body = np.linalg.det(C_inersia_to_body)
  print("det_C inersia to body= ", det_C_inersia_to_body)

  if det_C_inersia_to_body == 1:
      print("Has the terms of the transfer matrix")

      # Calculation Euler_angles
      TETHA = math.asin(-C_inersia_to_body[0, 2])
      COSTETHA = math.cos(TETHA)

      # Prevent division by zero (singularity)
      if abs(COSTETHA) > 1e-6:
          PSI = math.asin(C_inersia_to_body[0, 1] / COSTETHA)
          PHI = math.asin(C_inersia_to_body[1, 2] / COSTETHA)

          print("Euler_angles: ")
          print("TETHA= ", TETHA)
          print("PSI= ", PSI)
          print("PHI= ", PHI)

          #a b c d Quaternion_vector
          a=(math.cos(PHI/2)*math.cos(TETHA/2)*math.cos(PSI/2))-(math.sin(PHI/2)*math.sin(TETHA/2)*math.sin(PSI/2))
          b=(math.sin(PHI/2)*math.cos(TETHA/2)*math.cos(PSI/2))+(math.cos(PHI/2)*math.sin(TETHA/2)*math.sin(PSI/2))
          c=(math.cos(PHI/2)*math.sin(TETHA/2)*math.cos(PSI/2))-(math.sin(PHI/2)*math.cos(TETHA/2)*math.sin(PSI/2))
          d=(math.cos(PHI/2)*math.cos(TETHA/2)*math.sin(PSI/2))-(math.sin(PHI/2)*math.sin(TETHA/2)*math.cos(PSI/2))

          Quaternion_vector = np.array([[a],[b],[c],[d]])
          print("Quaternion_vector= ")
          print(Quaternion_vector)
          # Prevent division by zero (singularity)
          if abs(math.sin(PHI / 2)) > 1e-6:
              PHI_x = (b/math.sin(PHI/2))*PHI
              PHI_y = (c/math.sin(PHI/2))*PHI
              PHI_z = (d/math.sin(PHI/2))*PHI

              # Rotaion Vector
              RotaionVector = np.array([[PHI_x] [PHI_y][PHI_z]])
              print("Rotaion_Vector: ", RotaionVector)
          else:
              print("singularity")
      else:
          print("singularity")      

  else:
    print("The input matrix is not a valid rotation matrix")


if __name__ == "__main__":
    choice = input("شماره سوال: ")

    if choice == "1":
        exercise_1()
    elif choice == "2":
        exercise_2()
    elif choice == "3":
        exercise_3()
    else:
        print("انتخاب نادرست")