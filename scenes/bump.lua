--all primitives
--light source
--one shiny with phong
--additional feature

--puppet crystal ball?
--fractal snowflake

-- materials
stone = gr.material({0.8, 0.7, 0.7}, {0.0, 0.0, 0.0}, 0)
--grey_stone = gr.fancy_material({0.4, 0.4, 0.4}, {0.2, 0.2, 0.2}, 20, 0.5, 1.0, 0.0)
grass = gr.material({0.1, 0.0, 0.3}, {0.0, 0.0, 0.0}, 0)
ground = gr.material({0.3, 0.3, 0.3}, {0.0, 0.0, 0.0}, 0)
hide = gr.material({0.84, 0.6, 0.53}, {0.3, 0.3, 0.3}, 20)

glass = gr.fancy_material({1.0,1.0,1.0}, {0.1, 0.1, 0.1}, 50, 0.0, 1.01, 1, 0.0)
shirt = gr.material({0.9,0.9,0.9}, {0, 0, 0}, 2)
pants = gr.material({0.2,0.3,0.9}, {0, 0, 0}, 2)
shoes = gr.material({0.1,0.1,0.1}, {0.3, 0.3, 0.3}, 20)

white_cornell = gr.material({0.740063, 0.742313, 0.733934}, {0, 0, 0}, 0)
white_cornell_shiny = gr.fancy_material({0.740063/2, 0.742313/2, 0.733934/2}, {1, 1, 1}, 4, 0.6,1.0,0.0,0.3)
red_cornell =   gr.material({0.366046, 0.037182, 0.041638}, {0, 0, 0}, 0)
green_cornell = gr.material({0.162928, 0.408903, 0.083375}, {0, 0, 0}, 0)



scene = gr.node('scene')
mirror1 = gr.fancy_material({0,0,0}, {1, 1, 1}, 40000000.0, 0.0, 1.0, 0.0, 1)
mirror2 = gr.fancy_material({0,0,0}, {1, 1, 1}, 1000.0, 0.0, 1.0, 0.0, 20)
mirror3 = gr.fancy_material({0,0,0}, {1, 1, 1}, 100.0, 0.0, 1.0, 0.0, 40)

tex_test = gr.fancy_material({0.740063, 0.742313, 0.733934}, {0, 0, 0}, 10000.0, 0.8,1.0,0.0,1)
tex_test:set_texture_map("textures/grass/texture-light.png")
tex_test:set_bump_map("textures/grass/bump.png", 0.03)
--tex_test:set_bump_map("textures/bumptest.png")


--ballmat = gr.fancy_material({1.0,1.0,1.0}, {0.5, 0.5, 0.5}, 40000000.0, 0.0, 1.0, 0.0, 1)
ballmat = gr.fancy_material({1.0,1.0,1.0}, {0.4, 0.4, 0.4}, 100.0, 0.0, 1.0, 0.0, 1)
ballmat:set_bump_map("textures/golfball_bump.png", 0.05)

ball = gr.sphere("ball")
ball:set_material(ballmat)
ball:translate(0,1,0)
ball:rotate('y',45)
scene:add_child(ball)

--teemat = gr.material({1, 1, 1}, {0, 0, 0}, 0)
teemat = gr.fancy_material({1.0,1.0,1.0}, {0.3, 0.3, 0.3}, 2000.0, 0.0, 1.0, 0.0, 3)
tee = gr.obj_mesh("tee", "meshes/golf_tee.obj")
tee:set_material(teemat)
scene:add_child(tee)
-- Cornell Box
cornell_box = gr.node('cornell box')

box_width = 15.0
box_height = 10.0
box_length = 30.0
-- 	--floor
-- floor = gr.('floor')
-- floor:set_material(tex_test)
-- floor:translate(-7,-2.0,-7)
-- floor:scale(box_width,1,box_width)
-- cornell_box:add_child(floor)

floor = gr.plane('floor', 5.0)
floor:set_material(tex_test)
floor:translate(0,-1.0,0)
floor:scale(5,5,5)
--floor:scale(box_width,1,box_width)
cornell_box:add_child(floor)


scene:add_child(cornell_box)


-- lights
light_color = {0.780131 * 1, 0.780409 * 1, 0.775833 * 1}
light_color = {1,1,1}
light_color_2 = {0.780131/2, 0.780409/2, 0.775833/2}
-- on ceiling
light1 = gr.light({-4, 10 , -4}, light_color, {1, 0, 0})
-- by camera
light2 = gr.light({0, box_height - 3.0, -2}, light_color_2, {1, 0, 0})


sqlight = gr.rect_light({-300, 150, 0}, 80, 80, light_color, {1,0,0}, 5)


--good
gr.render(scene,
	  'bump.png', 1920, 1080, 4,
	  {-4, 1, -4}, {-0, -0.3, 0}, {0, 1, 0}, 60,
	  {0.15,0.15,0.15}, {sqlight}, "textures/sky4.png")

-- gr.render(scene,
-- 	  'bump.png', 700, 700, 2,
-- 	  {-4, 1, -4}, {-0, -1, 0}, {0, 1, 0}, 60,
-- 	  {0.15,0.15,0.15}, {sqlight}, "textures/sky4light.png")



