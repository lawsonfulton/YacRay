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
white_cornell_shiny = gr.fancy_material({0.740063/2, 0.742313/2, 0.733934/2}, {1, 1, 1}, 10000.0, 0.6,1.0,0.0,1)
mirror = gr.fancy_material({0,0,0}, {1, 1, 1}, 50, 1.0, 1.0, 0.0, 1)
red_cornell =   gr.material({0.366046, 0.037182, 0.041638}, {0, 0, 0}, 0)
green_cornell = gr.material({0.162928, 0.408903, 0.083375}, {0, 0, 0}, 0)


tex_test = gr.fancy_material({0.740063, 0.742313, 0.733934}, {0, 0, 0}, 10000000.0, 0.8,1.0,0.0,2)
tex_test:set_bump_map("textures/moonbump2.png", 0.05)

--tex_test:set_texture_map("textures/masonrytexture.png")
white_cornell_bump = gr.material({0.740063, 0.742313, 0.733934}, {0, 0, 0}, 0)
-- white_cornell_bump:set_texture_map("textures/masonrytexture.png")
white_cornell_bump:set_bump_map("textures/bumptest.png",0.2)
scene = gr.node('scene')



sphere = gr.sphere('s')
radius = 1.5
sphere:translate(0,0,0)
sphere:rotate('z',90)
sphere:set_material(tex_test)
scene:add_child(sphere)

-- box = gr.cube('c')
-- box:translate(1,-1,0)
-- box:scale(2,4,2)
-- box:rotate('y', 30)
-- box:set_material(white_cornell)
-- scene:add_child(box)

-- Cornell Box
cornell_box = gr.node('cornell box')

box_width = 10.0
box_height = 10.0
box_length = 30.0
	--floor
floor = gr.plane('floor', 30.0)
floor:set_material(white_cornell_bump)
floor:translate(5.0,-1.0,0)
floor:scale(3,3,3)
--floor:scale(box_width,1,box_width)
cornell_box:add_child(floor)

	--ceiling
ceiling = gr.cube('ceiling')
ceiling:set_material(white_cornell)
ceiling:translate(-box_width/2.0,box_height - 2.0,-box_length/1.5)
ceiling:scale(box_width,1,box_length)
cornell_box:add_child(ceiling)


	--left wall

left_wall = gr.plane('left_wall', 30.0)
left_wall:set_material(white_cornell_bump)
left_wall:translate(5,0,0)
left_wall:rotate('y',90)
left_wall:rotate('x',-90)

left_wall:scale(3,3,3)
--left_wall:scale(box_width,1,box_width)
cornell_box:add_child(left_wall)

-- left_wall = gr.cube('left_wall')
-- left_wall:set_material(green_cornell)
-- left_wall:translate(box_width/2.0 - 1,-1.0,-box_length/1.5)
-- left_wall:scale(1,box_height,box_length)
-- cornell_box:add_child(left_wall)

	--right wall
right_wall = gr.cube('right_wall')
right_wall:set_material(red_cornell)
right_wall:translate(-box_width/2.0,-1.0,-box_length/1.5)
right_wall:scale(1,box_height,box_length)
cornell_box:add_child(right_wall)

	--back wall
back_wall = gr.cube('back_wall')
back_wall:set_material(white_cornell)
back_wall:translate(-box_width/2.0,-1.0,box_length/6)
back_wall:scale(box_width,box_height,1.0)
cornell_box:add_child(back_wall)

--scene:add_child(cornell_box)


-- lights
light_color = {1,1,1}
light_color_2 = {0.780131/2, 0.780409/2, 0.775833/2}
-- on ceiling
light1 = gr.light({4, 3.0, -1}, light_color_2, {1, 0, 0})
-- by camera
light2 = gr.light({-2.0, box_height - 3.0, -3}, light_color_2, {1, 0, 0})


sqlight = gr.rect_light({0, 100, 0}, 10, 10, light_color, {1,0,0}, 10)


--far
gr.render(scene,
	  'moon.png', 700, 700,1,
	  {0, 0, 5}, {0, 0, 0}, {1, 0, 0}, 50,
	  {0,0,0}, {sqlight})


--close
-- gr.render(scene,
-- 	  'bump2.png', 700, 700, 2,
-- 	  {0, box_height/100.0, -box_length/100.0}, {0, -10, 30}, {0, 1, 0}, 50,
-- 	  {0.2,0.2,0.2}, {light1, light2})







