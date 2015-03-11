--all primitives
--light source
--one shiny with phong
--additional feature

--puppet crystal ball?
--fractal snowflake

-- materials
stone = gr.material({0.8, 0.7, 0.7}, {0.0, 0.0, 0.0}, 0)
grey_stone = gr.fancy_material({0.4, 0.4, 0.4}, {0.2, 0.2, 0.2}, 20, 0.5, 1.0, 0.0)
grass = gr.material({0.1, 0.0, 0.3}, {0.0, 0.0, 0.0}, 0)
ground = gr.material({0.3, 0.3, 0.3}, {0.0, 0.0, 0.0}, 0)
hide = gr.material({0.84, 0.6, 0.53}, {0.3, 0.3, 0.3}, 20)
mirror = gr.fancy_material({0.9,0.9,0.9}, {0.1, 0.1, 0.1}, 50, 0.8, 1.0, 0.0)
glass = gr.fancy_material({1.0,1.0,1.0}, {0.1, 0.1, 0.1}, 50, 0.2, 2.92, 1)
shirt = gr.material({0.9,0.9,0.9}, {0, 0, 0}, 2)
pants = gr.material({0.2,0.3,0.9}, {0, 0, 0}, 2)
shoes = gr.material({0.1,0.1,0.1}, {0.3, 0.3, 0.3}, 20)

-- functions
function sphere_snowflake(x,y,z, material, level, max_level, skip_index)
	local snowflake = gr.node('snowflake')

	base = 3.0
	local radius = 1.0 / (base^level)
	local offset = radius + 1.0 / (base^(level+1))

	local sphere = gr.nh_sphere('s', {x,y,z}, radius)
	snowflake:add_child(sphere)
	sphere:set_material(material)

	if(level < max_level) then
		if(skip_index ~= 1) then
			local flake = sphere_snowflake(x + offset, y, z, material, level + 1, max_level, 4)
			snowflake:add_child(flake)
		end
		if(skip_index ~= 2) then
			local flake = sphere_snowflake(x, y + offset, z, material, level + 1, max_level, 5)
			snowflake:add_child(flake)
		end
		if(skip_index ~= 3) then
			local flake = sphere_snowflake(x, y, z + offset, material, level + 1, max_level, 6)
			snowflake:add_child(flake)
		end
		if(skip_index ~= 4) then
			local flake = sphere_snowflake(x - offset, y, z, material, level + 1, max_level, 1)
			snowflake:add_child(flake)
		end
		if(skip_index ~= 5) then
			local flake = sphere_snowflake(x, y - offset, z, material, level + 1, max_level, 2)
			snowflake:add_child(flake)
		end
		if(skip_index ~= 6) then
			local flake = sphere_snowflake(x, y, z - offset, material, level + 1, max_level, 3)
			snowflake:add_child(flake)
		end
	end

	return snowflake
end

-- Puppet Definition

puppet = gr.node('root')
rotateNode = gr.node('rotateNode')

torsoNode = gr.node("torsoNode")
shouldersNode = gr.node("shouldersNode")
hipsNode = gr.node("hipsNode")
leftUpperArmNode = gr.node("leftUpperArmNode")
rightUpperArmNode = gr.node("rightUpperArmNode")
leftThighNode = gr.node("leftThighNode")
rightThighNode = gr.node("rightThighNode")
neckNode = gr.node("neckNode")
leftForearmNode = gr.node("leftForearmNode")
rightForearmNode = gr.node("rightForearmNode")
leftCalfNode = gr.node("leftCalfNode")
rightCalfNode = gr.node("rightCalfNode")
headNode = gr.node("headNode")
noseNode = gr.node("noseNode")
eyeNode = gr.node("eyeNode")
leftHandNode = gr.node("leftHandNode")
rightHandNode = gr.node("rightHandNode")
leftFootNode = gr.node("leftFootNode")
rightFootNode = gr.node("rightFootNode")

torso = gr.sphere("torso")
shoulders = gr.sphere("shoulders")
hips = gr.sphere("hips")
leftUpperArm = gr.sphere("leftUpperArm")
rightUpperArm = gr.sphere("rightUpperArm")
leftThigh = gr.sphere("leftThigh")
rightThigh = gr.sphere("rightThigh")
neck = gr.sphere("neck")
nose = gr.sphere("nose")
leftEye = gr.sphere("leftEye")
rightEye = gr.sphere("rightEye")
leftForearm = gr.sphere("leftForearm")
rightForearm = gr.sphere("rightForearm")
leftCalf = gr.sphere("leftCalf")
rightCalf = gr.sphere("rightCalf")
head = gr.sphere("head")
leftHand = gr.sphere("leftHand")
rightHand = gr.sphere("rightHand")
leftFoot = gr.sphere("leftFoot")
rightFoot = gr.sphere("rightFoot")

upperNeckJoint = gr.joint("upperNeckJoint", {0,-20,360}, {0,0,360})
lowerNeckJoint = gr.joint("lowerNeckJoint", {0,0,360}, {0,0,360})
leftShoulderJoint = gr.joint("leftShoulderJoint", {0,40,360}, {0,0,360})
rightShoulderJoint = gr.joint("rightShoulderJoint", {0,40,360}, {0,0,360})
leftElbowJoint = gr.joint("leftElbowJoint", {0,70,360}, {0,0,360})
rightElbowJoint = gr.joint("rightElbowJoint", {0,70,360}, {0,0,360})
leftWristJoint = gr.joint("leftWristJoint", {0,0,360}, {0,0,360})
rightWristJoint = gr.joint("rightWristJoint", {0,0,360}, {0,0,360})
leftHipJoint = gr.joint("leftHipJoint", {0,60,360}, {0,0,360})
rightHipJoint = gr.joint("rightHipJoint", {0,60,360}, {0,0,360})
leftKneeJoint = gr.joint("leftKneeJoint", {0,-50,360}, {0,0,360})
rightKneeJoint = gr.joint("rightKneeJoint", {0,-50,360}, {0,0,360})
leftAnkleJoint = gr.joint("leftAnkleJoint", {0,0,360}, {0,0,360})
rightAnkleJoint = gr.joint("rightAnkleJoint", {0,0,360}, {0,0,360})


torso:set_material(shirt);
shoulders:set_material(shirt);
hips:set_material(shirt);
leftUpperArm:set_material(hide);
rightUpperArm:set_material(hide);
leftThigh:set_material(pants);
rightThigh:set_material(pants);
neck:set_material(hide);
leftForearm:set_material(hide);
rightForearm:set_material(hide);
leftCalf:set_material(pants);
rightCalf:set_material(pants);
head:set_material(hide);
leftHand:set_material(hide);
rightHand:set_material(hide);
leftFoot:set_material(shoes);
rightFoot:set_material(shoes);
nose:set_material(hide);
leftEye:set_material(shoes);
rightEye:set_material(shoes);

puppet:add_child(rotateNode)
rotateNode:add_child(torsoNode)
torsoNode:add_child(torso)
torso:scale(1,1.5,0.5)

-- torso
torsoNode:add_child(shouldersNode)
shouldersNode:translate(0,1.3,0)
shouldersNode:add_child(shoulders)
shoulders:scale(1.3,0.3,0.5)

-- neck
torsoNode:add_child(lowerNeckJoint)
lowerNeckJoint:translate(0, 1.5, 0)

lowerNeckJoint:add_child(neckNode)
neckNode:translate(0,0.1,0)
neckNode:add_child(neck)
neck:scale(0.2,0.4,0.2)

neckNode:add_child(upperNeckJoint)
upperNeckJoint:translate(0, 0.4, 0)

-- head
upperNeckJoint:add_child(headNode)
headNode:translate(0,0.4,0)
headNode:add_child(head)
head:scale(0.7,0.7,0.7)

headNode:add_child(noseNode)
noseNode:translate(0,-0.2,-0.7)
noseNode:add_child(nose)
nose:scale(0.1,0.1,0.1)

eyeSize = 0.18
headNode:add_child(eyeNode)
eyeNode:add_child(leftEye)
leftEye:translate(-0.2,0,-0.7)
leftEye:scale(eyeSize, eyeSize, eyeSize)

eyeNode:add_child(rightEye)
rightEye:translate(0.2,0,-0.7)
rightEye:scale(eyeSize, eyeSize, eyeSize)

-- arms
shouldersNode:add_child(leftShoulderJoint)
leftShoulderJoint:translate(-1.3,0,0)
leftShoulderJoint:add_child(leftUpperArmNode)
leftUpperArmNode:translate(0,-1,0)
leftUpperArmNode:add_child(leftUpperArm)
leftUpperArm:scale(0.2,1,0.2)

shouldersNode:add_child(rightShoulderJoint)
rightShoulderJoint:translate(1.3,0,0)
rightShoulderJoint:add_child(rightUpperArmNode)
rightUpperArmNode:translate(0,-1,0)
rightUpperArmNode:add_child(rightUpperArm)
rightUpperArm:scale(0.2,1,0.2)

leftUpperArmNode:add_child(leftElbowJoint)
leftElbowJoint:translate(0,-1,0)
leftElbowJoint:add_child(leftForearmNode)
leftForearmNode:translate(0,-0.8,0)
leftForearmNode:add_child(leftForearm)
leftForearm:scale(0.15,0.8,0.15)

rightUpperArmNode:add_child(rightElbowJoint)
rightElbowJoint:translate(0,-1,0)
rightElbowJoint:add_child(rightForearmNode)
rightForearmNode:translate(0,-0.8,0)
rightForearmNode:add_child(rightForearm)
rightForearm:scale(0.15,0.8,0.15)

leftForearmNode:add_child(leftWristJoint)
leftWristJoint:translate(0,-0.8,0)
leftWristJoint:add_child(leftHandNode)
leftHandNode:translate(0,-0.1,0)
leftHandNode:add_child(leftHand)
leftHand:scale(0.1,0.25,0.2)

rightForearmNode:add_child(rightWristJoint)
rightWristJoint:translate(0,-0.8,0)
rightWristJoint:add_child(rightHandNode)
rightHandNode:translate(0,-0.1,0)
rightHandNode:add_child(rightHand)
rightHand:scale(0.1,0.25,0.2)

-- legs
torsoNode:add_child(hipsNode)
hipsNode:translate(0,-1.3,0)
hipsNode:add_child(hips)
hips:scale(1,0.4,0.5)

hipsNode:add_child(leftHipJoint)
leftHipJoint:translate(-0.7,0,0)
leftHipJoint:add_child(leftThighNode)
leftThighNode:translate(0,-1,0)
leftThighNode:add_child(leftThigh)
leftThigh:scale(0.3,1.3,0.3)

hipsNode:add_child(rightHipJoint)
rightHipJoint:translate(0.7,0,0)
rightHipJoint:add_child(rightThighNode)
rightThighNode:translate(0,-1,0)
rightThighNode:add_child(rightThigh)
rightThigh:scale(0.3,1.3,0.3)

leftThighNode:add_child(leftKneeJoint)
leftKneeJoint:translate(0,-1,0)
leftKneeJoint:add_child(leftCalfNode)
leftCalfNode:translate(0,-1,0)
leftCalfNode:add_child(leftCalf)
leftCalf:scale(0.25,1,0.25)

rightThighNode:add_child(rightKneeJoint)
rightKneeJoint:translate(0,-1,0)
rightKneeJoint:add_child(rightCalfNode)
rightCalfNode:translate(0,-1,0)
rightCalfNode:add_child(rightCalf)
rightCalf:scale(0.25,1,0.25)

leftCalfNode:add_child(leftAnkleJoint)
leftAnkleJoint:translate(0,-0.8,0)
leftAnkleJoint:add_child(leftFootNode)
leftFootNode:translate(0,-0.1,-0.1)
leftFootNode:add_child(leftFoot)
leftFoot:scale(0.2,0.2,0.5)

rightCalfNode:add_child(rightAnkleJoint)
rightAnkleJoint:translate(0,-0.8,0)
rightAnkleJoint:add_child(rightFootNode)
rightFootNode:translate(0,-0.1,-0.1)
rightFootNode:add_child(rightFoot)
rightFoot:scale(0.2,0.2,0.5)

------------ main scene ---------------
scene = gr.node('scene')

snowflake = sphere_snowflake(0,0,0, mirror, 0, 3, 7)
scene:add_child(snowflake)
snowflake:translate(-3,0.3,-2)
snowflake:rotate('x',-40)
snowflake:rotate('z',45)
-- snowflake:rotate('x',45)
--snowflake:rotate('y',45)
-- snowflake:rotate('y',45)
-- snowflake:rotate('x',45)

--puppet
scene:add_child(puppet)
puppet:translate(3,3.8,0)
puppet:rotate("y", 45)
-- puppet:translate(0,3.8,0)
-- puppet:rotate("y", 10)
rotateNode:rotate("x",-20)

--crystal ball
	--ball
ball_stand = gr.node("stand")
ball = gr.sphere('ball')
ball_stand:add_child(ball)
ball:set_material(glass)
ball:translate(0,-0.3,-3)
ball:scale(0.8,0.8,0.8)

	--base
base = gr.cube("base")
ball_stand:add_child(base)
base:set_material(grey_stone)
base:translate(-0.46,-5.05,-3.6)
base:scale(1,4,1)

require('cylinder')

ball_stand:add_child(cyl)
cyl:set_material(grey_stone)
cyl:translate(-0.05,-5,-3)
cyl:scale(1.3, 1, 1.3)

puppet:add_child(ball_stand)

-- the floor
plane = gr.mesh('plane', {
		   { -1, 0, -1 }, 
		   {  1, 0, -1 }, 
		   {  1,  0, 1 }, 
		   { -1, 0, 1  }
		}, {
		   {3, 2, 1, 0}
		})
scene:add_child(plane)
plane:set_material(ground)
plane:translate(0,-1,0)
plane:scale(300, 300, 300)

-- cieling
plane = gr.mesh('plane', {
		   { -1, 0, -1 }, 
		   {  1, 0, -1 }, 
		   {  1,  0, 1 }, 
		   { -1, 0, 1  }
		}, {
		   {3, 2, 1, 0}
		})
scene:add_child(plane)
plane:set_material(ground)
plane:translate(0,25,0)
plane:rotate('x',180)
plane:scale(300, 300, 300)

plane = gr.mesh('plane', {
		   { -1, 0, -1 }, 
		   {  1, 0, -1 }, 
		   {  1,  0, 1 }, 
		   { -1, 0, 1  }
		}, {
		   {3, 2, 1, 0}
		})
scene:add_child(plane)
plane:set_material(grass)
plane:translate(0,0,10)
plane:rotate('x', -90)
plane:scale(300, 300, 300)

plane = gr.mesh('plane', {
		   { -1, 0, -1 }, 
		   {  1, 0, -1 }, 
		   {  1,  0, 1 }, 
		   { -1, 0, 1  }
		}, {
		   {3, 2, 1, 0}
		})
scene:add_child(plane)
plane:set_material(grass)
plane:translate(8,0,0)
plane:rotate('y', -90)
plane:rotate('x', -90)

plane:scale(300, 300, 300)

-- lights
-- light1 = gr.light({-20, 20, -30}, {0.5, 0.5, 0.5}, {1, 0, 0})
light1 = gr.light({-3, 3, -2}, {2.5, 2.5, 2.5}, {1, 0, 0.1})
point = {1.4,4.8,-2.4}
light2 = gr.light(point, {0.5, 0.5, 0.5}, {1, 0, 0})
s = gr.nh_sphere('s', point, 0.1)
-- scene:add_child(s)
s:set_material(stone)
-- render

--far
-- gr.render(scene,
-- 	  'sample.png', 800, 800,
-- 	  {-4, 4, -20}, {2, -2, 10}, {0, 1, 0}, 50,
-- 	  {0.2, 0.2, 0.2}, {light1, light2})

-- --close
-- gr.render(scene,
-- 	  'sample.png', 800, 800,
-- 	  {-2, 4, -10}, {2, -2, 10}, {0, 1, 0}, 50,
-- 	  {0.2, 0.2, 0.2}, {light1, light2})
gr.render(scene,
	  'sample.png', 600, 600,
	  {-2, 4, -12}, {1.5, -2, 10}, {0, 1, 0}, 50,
	  {0.2, 0.2, 0.2}, {light1, light2})

--top
--close
-- gr.render(scene,
-- 	  'sample.png', 800, 800,
-- 	  {0, 10, 0}, {0, -9, -5}, {0, 1, 0}, 50,
-- 	  {0.2, 0.2, 0.2}, {light1, light2})



