scene = gr.node('scene')


tex_test = gr.fancy_material({0.740063, 0.742313, 0.733934}, {0.4, 0.4, 0.4}, 1000.0, 0.8,1.0,0.0,1)
tex_test:set_texture_map("textures/wood_floor.png")

cup = gr.fancy_material({0.02, 0.01, 0.4}, {0.9, 0.9, 0.9}, 100000000000000.0, 0.8,1.52,0,1)

ballx = -1
bally = -1
ballz = 1
bunny = gr.obj_mesh("bunny", "meshes/dragon_smooth.obj")
bunny:translate(1,-1,2)
--bunny:scale(8,8,8)
bunny:rotate('y',200)
bunny:set_material(cup)
scene:add_child(bunny)


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

scene:add_child(cornell_box)


-- lights
light_color = {0.780131 * 1.0, 0.780409 * 1.0, 0.775833 * 1.0}
light_color_2 = {0.780131/2, 0.780409/2, 0.775833/2}
-- on ceiling
light1 = gr.light({-3, box_height - 3.0, -3}, light_color_2, {1, 0, 0})
-- by camera
light2 = gr.light({ballx - 10,5, 10}, light_color, {1, 0, 0})


sqlight = gr.rect_light({5,15, -10}, 5, 5, light_color, {1,0,0}, 5)
sqlight2 = gr.rect_light({-5,15, -10}, 5, 5, light_color, {1,0,0}, 5)

-- 	  {0.2, 0.2, 0.2}, {light1, light2})

-- --close

-- gr.render(scene,
-- 	  'kd.png', 1280, 720, 1,
-- 	  {-3, 0.5, -2}, {ballx, bally + 0.5, ballz}, {0, 1, 0}, 50,
-- 	  {0.15,0.15,0.15}, {sqlight}, "textures/apartment_env_map_sm.png")


gr.render(scene,
	  'kd.png', 700, 700, 4,
	  {-1, 0.5, -2}, {0, 0, 0}, {0, 1, 0}, 50,
	  {0.15,0.15,0.15}, {sqlight,sqlight2}, "textures/apartment_env_map_sm.png")


--top
--close
-- gr.render(scene,
-- 	  'reflect.png', 800, 800,
-- 	  {0, 10, 0}, {0, -9, -5}, {0, 1, 0}, 50,
-- 	  {0.2, 0.2, 0.2}, {light1, light2})



