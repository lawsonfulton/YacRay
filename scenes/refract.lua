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

--glass = gr.fancy_material({0,0,0}, {0, 0, 0}, 5000, 1, 1.52, 1.0, 1)
glass = gr.fancy_material({0,0,0}, {0.1, 0.1, 0.1}, 50000000.0, 1, 1.52, 1.0, 1)

-- glass = gr.fancy_material({0.8,0.9,0.8}, {0.2, 0.2, 0.2}, 50000000.0, 1, 1.52, 0.0, 1)

tex_test = gr.fancy_material({0.740063, 0.742313, 0.733934}, {0.4, 0.4, 0.4}, 1000.0, 0.8,1.0,0.0,10)
tex_test:set_texture_map("textures/wood_floor.png")



checker = gr.fancy_material({0.740063, 0.742313, 0.733934}, {1, 1, 1}, 4, 0,1.0,0.0,0.3)
checker:set_texture_map("textures/big_checker.png")

-- require('readobj')


ballx = -1
bally = 1
ballz = 1
ball = gr.sphere("ball")
ball:translate(ballx,bally,ballz)
ball:set_material(glass)
scene:add_child(ball)

mesh = gr.sphere("asff")--gr.obj_mesh('test_mesh','meshes/uv_sphere.obj')
mesh:translate(ballx,bally-2,ballz)
-- --mesh:scale(3,3,3)
-- mesh:rotate('y',180)
mesh:set_material(tex_test)
scene:add_child(mesh)

-- box = gr.cube('c')
-- box:translate(1,-1,0)
-- box:scale(2,4,2)
-- box:rotate('y', 30)
-- box:set_material(checker)
-- scene:add_child(box)

-- Cornell Box
cornell_box = gr.node('cornell box')

box_width = 15.0
box_height = 10.0
box_length = 30.0
-- 	--floor
floor = gr.cube('floor')
floor:set_material(tex_test)
floor:translate(-7,-2.0,-7)
floor:scale(box_width,1,box_width)
cornell_box:add_child(floor)


-- 	--ceiling
-- ceiling = gr.cube('ceiling')
-- ceiling:set_material(white_cornell)
-- ceiling:translate(-box_width/2.0,box_height - 2.0,-box_length/1.5)
-- ceiling:scale(box_width,1,box_length)
-- cornell_box:add_child(ceiling)


-- 	--left wall
-- left_wall = gr.cube('left_wall')
-- left_wall:set_material(green_cornell)
-- left_wall:translate(box_width/2.0 - 1,-1.0,-box_length/1.5)
-- left_wall:scale(1,box_height,box_length)
-- cornell_box:add_child(left_wall)

-- 	--right wall
-- right_wall = gr.cube('right_wall')
-- right_wall:set_material(red_cornell)
-- right_wall:translate(-box_width/2.0,-1.0,-box_length/1.5)
-- right_wall:scale(1,box_height,box_length)
-- cornell_box:add_child(right_wall)

-- 	--back wall
-- back_wall = gr.cube('back_wall')
-- back_wall:set_material(white_cornell)
-- back_wall:translate(-box_width/2.0,-1.0,box_length/6)
-- back_wall:scale(box_width,box_height,1.0)
-- cornell_box:add_child(back_wall)

scene:add_child(cornell_box)


-- lights
light_color = {0.780131 * 1.8, 0.780409 * 1.8, 0.775833 * 1.8}
light_color_2 = {0.780131/2, 0.780409/2, 0.775833/2}
-- on ceiling
light1 = gr.light({-3, box_height - 3.0, -3}, light_color_2, {1, 0, 0})
-- by camera
light2 = gr.light({ballx - 10,5, 10}, light_color, {1, 0, 0})


sqlight = gr.rect_light({ballx - 10,5, 10}, 3, 3, light_color, {1,0,0}, 40)


-- 	  {0.2, 0.2, 0.2}, {light1, light2})

-- --close
-- gr.render(scene,
-- 	  'reflect.png', 800, 800,
-- 	  {-2, 4, -10}, {2, -2, 10}, {0, 1, 0}, 50,
-- 	  {0.2, 0.2, 0.2}, {light1, light2})
gr.render(scene,
	  'refract.png', 700, 700, 4,
	  {1, 0.5, -5}, {ballx, bally, ballz}, {0, 1, 0}, 50,
	  {0.15,0.15,0.15}, {sqlight}, "textures/apartment_env_map_sm.png")

--top
--close
-- gr.render(scene,
-- 	  'reflect.png', 800, 800,
-- 	  {0, 10, 0}, {0, -9, -5}, {0, 1, 0}, 50,
-- 	  {0.2, 0.2, 0.2}, {light1, light2})


