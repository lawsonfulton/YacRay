scene = gr.node('scene')




--Apple
appleMat = gr.fancy_material({0.9, 0.9, 0.9}, {0.7, 0.7, 0.7}, 100.0, 0.8,1.0,0.0,5)
appleMat:set_texture_map("meshes/apple/skin.png")
stemMat = gr.fancy_material({0.740063, 0.742313, 0.733934}, {0, 0, 0}, 400.0, 0.8,1.0,0.0,1)
stemMat:set_texture_map("meshes/apple/stem.png")

apple = gr.obj_mesh("apple", "meshes/apple/apple_smooth.obj")
apple:scale(0.05,0.05,0.05)
apple:translate(-0.3, 0, -1)
apple:rotate('y', -45)
apple:set_material(appleMat)
scene:add_child(apple)

stem = gr.obj_mesh("stem", "meshes/apple/stem_smooth.obj")
stem:set_material(stemMat)
apple:add_child(stem)

--Golf Ball
ballmat = gr.fancy_material({0.6,0.6,0.6}, {0.8, 0.8, 0.8}, 500.0, 0.0, 1.0, 0.0, 5)
ballmat:set_bump_map("textures/golfball_bump.png", 0.0007)

ball = gr.sphere("ball")
ball:set_material(ballmat)
ball:scale(0.02,0.02,0.02)
ball:translate(-5,1,3)
ball:rotate('y',45)
ball:rotate('x',-45)
scene:add_child(ball)

--Water Bottle

bottleWaterMat = gr.fancy_material({0.0, 0.0, 0.0}, {0.2, 0.2, 0.2}, 2000000.0, 0.8,1.33,0.99,1)

bottleWater = gr.obj_mesh("bottleWater", "meshes/bottle/bottle_water.obj")
bottleWater:scale(0.05,0.05,0.05)
bottleWater:translate(2, 0, 0) 
bottleWater:rotate('y',-45)
bottleWater:set_material(bottleWaterMat)
scene:add_child(bottleWater)

bottleCapMat = gr.fancy_material({0.3, 0.3, 0.8}, {0.2, 0.2, 0.2}, 200.0, 0.8,1.33,0.0,3)

bottleCap = gr.obj_mesh("bottleCap", "meshes/bottle/bottle_cap.obj")
bottleCap:set_material(bottleCapMat)
bottleWater:add_child(bottleCap)

bottleLabelMat = gr.fancy_material({1,1,1}, {0.1, 0.1, 0.1}, 200.0, 0.8,1.33,0.0,3)
bottleLabelMat:set_texture_map("meshes/bottle/label.png")

bottleLabel = gr.obj_mesh("bottleLabel", "meshes/bottle/bottle_label.obj")
bottleLabel:set_material(bottleLabelMat)
bottleWater:add_child(bottleLabel)


-- --pointer
pointerMat = gr.fancy_material({0.2, 0.2, 0.2}, {1, 1, 1}, 20000000000.0, 0.8,1.0,0.0,1)

require('readobj')
pointer = gr.obj_mesh("pointer", "meshes/pointer.obj")
--pointer = gr.mesh('cow', readobj('meshes/pointer.obj'))
pointer:scale(0.05,0.05,0.05)
pointer:translate(-0.5, 0, 0.5) 
pointer:rotate('y', 150)
pointer:set_material(pointerMat)
scene:add_child(pointer)

--desk
deskMat = gr.fancy_material({1, 1, 1}, {1, 1, 1}, 1000.0, 0.8,1.0,0.0,8)
deskMat:set_texture_map("textures/planks/wood_diffuse_sm.png")
deskMat:set_specular_map("textures/planks/wood_specular_sm.png")
deskMat:set_bump_map("textures/planks/wood_bump_sm.png", 0.01)
--mirror = gr.fancy_material({0, 0, 0}, {1, 1, 1}, 100000000.0, 0.8,1.0,0.0,1)

desk = gr.plane('desk', 10)
desk:set_material(deskMat)
desk:translate(0,0.0,3.5)
--desk:scale(15,15,15)
desk:rotate('y',90)
scene:add_child(desk)

--TODO test without fresnel
-- lights
light_color = {0.780131 * 1.8, 0.780409 * 1.8, 0.775833 * 1.8}
sqlight = gr.rect_light({5, 7, 3}, 3, 3, light_color, {1,0,0}, 5)
-- sqlight = gr.rect_light({0, 0.5, -2}, 1, 1, light_color, {1,0,0}, 5)
-- sqlight = gr.light({-4, 10 , -4}, light_color, {1, 0, 0})

zoom = 10
gr.render(scene,
	  'final.png', 4000, 4000, 4,
	  {1/zoom, 1.8/zoom, 3/zoom}, {0, -0.03, 0}, {0, 1, 0}, 50,
	  {0.15,0.15,0.15}, {sqlight}, "textures/apartment_env_map_sm.png")



