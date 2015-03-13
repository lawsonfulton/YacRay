require('readobj')

gold = gr.material({0.9, 0.8, 0.4}, {0.8, 0.8, 0.4}, 25)
grass = gr.material({0.1, 0.7, 0.1}, {0.0, 0.0, 0.0}, 0)
blue = gr.material({0.7, 0.6, 1}, {0.5, 0.4, 0.8}, 25)

scene = gr.node('scene')


-- the floor
plane = gr.mesh('plane', {
	{-1, 0, -1},
	{ 1, 0, -1},
	{1,  0, 1},
	{-1, 0, 1},
     }, {
	{3, 2, 1, 0}
     })
scene:add_child(plane)
plane:set_material(grass)
plane:scale(30, 30, 30)

-- sphere
verts, faces = readobj('suzanne.obj')
poly = gr.mesh('poly',verts, faces)
scene:add_child(poly)
poly:set_material(blue)
poly:translate(0,1,0)
poly:rotate('x', 90)

-- The lights
l1 = gr.light({5,5,10}, {0.8, 0.8, 0.8}, {1, 0, 0})
l2 = gr.light({0, 5, -20}, {0.4, 0.4, 0.8}, {1, 0, 0})

gr.render(scene, 'suzy.png', 600, 600, 
	  {5, 5, 5}, {-3, -3, -3}, {0, 1, 0}, 50,
	  {0.4, 0.4, 0.4}, {l1, l2})
