import math

# Dimentions in mm
lengthRearArm = 75.0
lengthFrontArm = 77.5
# Horizontal distance from Joint3 to the center of the tool mounted on the end effector.
distanceTool = 85.0
# Joint1 height.
heightFromGround = 93.0

lengthRearSquared = pow(lengthRearArm, 2)
lengthFrontSquared = pow(lengthFrontArm, 2)

armSquaredConst = pow(lengthRearArm, 2) + pow(lengthFrontArm, 2)
armDoubledConst = 2.0 * lengthRearArm * lengthFrontArm
radiansToDegrees = 180.0 / math.pi
degreesToRadians = math.pi / 180.0

piHalf = math.pi / 2.0
piTwo = math.pi * 2.0
piThreeFourths = math.pi * 3.0 / 4.0

class DobotKinematics:
	def __init__(self, debug=False):
		self._debugOn = debug

	# def _debug(self, *args):
	# 	if #On:
	# 		# Since "print" is not a function the expansion (*) cannot be used
	# 		# as it is not an operator. So this is a workaround.
	# 		for arg in args:
	# 			sys.stdout.write(str(arg))
	# 			sys.stdout.write(' ')
	# 		print('')

	def coordinatesFromAngles(self, baseAngle, rearArmAngle, frontArmAngle):
		radius = lengthRearArm * math.cos(math.pi / 180.0*rearArmAngle) + lengthFrontArm * math.cos(math.pi / 180.0*frontArmAngle) + distanceTool
		x = radius * math.cos(math.pi / 180.0*baseAngle)
		y = radius * math.sin(math.pi / 180.0*baseAngle)
		z = heightFromGround - lengthFrontArm * math.sin(math.pi / 180.0*frontArmAngle) + lengthRearArm * math.sin(math.pi / 180.0*rearArmAngle)

		return (x, y, z)

	def anglesFromCoordinates(self, x, y, z):
		# Radius to the center of the tool.
		radiusTool = math.sqrt(pow(x, 2) + pow(y, 2))
		#('radiusTool', radiusTool)
		# Radius to joint3.
		radius = radiusTool - distanceTool
		#('radius', radius)
		baseAngle = math.atan2(y, x)
		#('ik base angle', baseAngle)
		# X coordinate of joint3.
		jointX = radius * math.cos(baseAngle)
		#('jointX', jointX)
		# Y coordinate of joint3.
		jointY = radius * math.sin(baseAngle)
		#('jointY', jointY)
		actualZ = z - heightFromGround
		#('actualZ', actualZ)
		# Imaginary segment connecting joint1 with joint2, squared.
		hypotenuseSquared = pow(actualZ, 2) + pow(radius, 2)
		hypotenuse = math.sqrt(hypotenuseSquared)
		#('hypotenuse', hypotenuse)
		#('hypotenuseSquared', hypotenuseSquared)

		q1 = math.atan2(actualZ, radius)
		#('q1', q1)
		q2 = math.acos((lengthRearSquared - lengthFrontSquared + hypotenuseSquared) / (2.0 * lengthRearArm * hypotenuse))
		#('q2', q2)
		rearAngle = piHalf - (q1 + q2)
		#('ik rear angle', rearAngle)
		frontAngle = piHalf - (math.acos((lengthRearSquared + lengthFrontSquared - hypotenuseSquared) / (2.0 * lengthRearArm * lengthFrontArm)) - rearAngle)
		#('ik front angle', frontAngle)

		return (baseAngle, rearAngle, frontAngle)

	def get_distance_from_origin_to_cartesian_point_3D(self, x, y, z):
		#get distance from origin (0,0,0) to end point in 3D using pythagorean thm in 3D; distance = sqrt(x^2+y^2+z^2)
		distanceToEndPoint = math.sqrt( pow(x,2) + pow(y,2) + pow(z,2) )
		return distanceToEndPoint

	# angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
	# i.e. they should be set up the same way as the unit circle is
	def check_for_angle_limits_is_valid(self, baseAngle, rearArmAngle, foreArmAngle):
		ret = True
		# implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
		# necessary for this function.
		# Not currently checking the base angle

		# check the rearArmAngle
		# max empirically determined to be around 107 - 108 degrees. Using 105.
		# min empirically determined to be around -23/24 degrees. Using -20.
		if (-44.5 > rearArmAngle > 90):
			print('Rear arm angle out of range')
			ret = False

		# check the foreArmAngle
		# the valid forearm angle is dependent on the rear arm angle. The real world angle of the forearm
		# (0 degrees = horizontal) needs to be evaluated.
		# min empirically determined to be around -105 degrees. Using -102.
		# max empirically determined to be around 21 degrees. Using 18.
		if (-102 > foreArmAngle > 18):
			print('Fore arm angle out of range')
			ret = False

		return ret

dobot = DobotKinematics()
while True:
	v = int(input("1(углы на вход) or 2(корды на вход) or 3(exit): "))
	if v == 1:
		"""Координаты из углов"""
		a = float(input("ВВедите значение углов поворота(база): "))
		b = float(input("ВВедите значение углов поворота(первое звено): "))
		c = float(input("ВВедите значение углов поворота(второе звено): "))
		flag = dobot.check_for_angle_limits_is_valid(a, b, c)
		if flag == True:
			x, y, z = dobot.coordinatesFromAngles(a, b, c)
			print("{:.2f}".format(x), "{:.2f}".format(y), "{:.2f}".format(z))
		else:
			print("Углы за пределами допучтимых")	
	elif v == 2:
		"""Углы из координат"""
		a = float(input("ВВедите значение координаты x: "))
		b = float(input("ВВедите значение координаты у: "))
		c = float(input("ВВедите значение координаты z: "))
		Ab, Ar, Af = dobot.anglesFromCoordinates(a, b, c)
		flag = dobot.check_for_angle_limits_is_valid(Ab, Ar, Af)
		if flag == True:
			print(Ab * (180.0 / math.pi), Ar * (180.0 / math.pi), Af * (180.0 / math.pi))
		else:
			print("Координаты за пределами допуcтимых")
	elif v == 3:
		break