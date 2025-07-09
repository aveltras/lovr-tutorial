phywire = require 'lovr-phywire/phywire'

local button = {
  text = 'RESTART',
  textSize = .1,
  count = 0,
  position = lovr.math.newVec3(0, 1, -3),
  width = 1.0,
  height = .4,
  hover = false,
  active = false
}

local hands = { -- palms that can push and grab objects
  colliders = {nil, nil},     -- physical objects for palms
  touching  = {nil, nil},     -- the collider currently touched by each hand
  holding   = {nil, nil},     -- the collider attached to palm
  solid     = {false, false}, -- hand can either pass through objects or be solid
}

local hand_torque = 10
local hand_force = 300

local tips = {}

local virtual_head_pose = Mat4()

local motion = {
    pose = lovr.math.newMat4(), -- Transformation in VR initialized to origin (0,0,0) looking down -Z
    thumbstickDeadzone = 0.4,   -- Smaller thumbstick displacements are ignored (too much noise)
    directionFrom = 'head',     -- Movement can be relative to orientation of head or left controller
    flying = false,
    -- Snap motion parameters
    snapTurnAngle = 2 * math.pi / 12,
    dashDistance = 1.5,
    thumbstickCooldownTime = 0.3,
    thumbstickCooldown = 0,
    -- Smooth motion parameters
    turningSpeed = 2 * math.pi * 1 / 6,
    walkingSpeed = 4,
}

local vrController = {
    -- state
    origin = lovr.math.newMat4(),
    -- settings
    thumbstickDeadzone = 0.4,
    walkingSpeed = 4,
    runningSpeed = 7,
}


function lovr.load()
    lovr.graphics.setBackgroundColor(0.208, 0.208, 0.275)
    world = lovr.physics.newWorld({
        tags = { "character", "leftHand", "rightHand", "pickupable" }
    })
    world:setGravity(0, -9.8, 0)

    gunshot = lovr.audio.newSource('gunshot.wav')

    -- layer = lovr.headset.newLayer(500, 500)
    -- layer:setViewport(0, 0, 500, 500)
    -- layer:setPosition(0, 2, -10)

    camera = lovr.graphics.newTexture(500, 500)
    cameraPass = lovr.graphics.newPass(camera)
    cameraPass:setClear(0xff00ff)

    
    -- display_height = lovr.headset.getDisplayHeight()
    -- display_width = lovr.headset.getDisplayWidth()
    -- layer = lovr.headset.newLayer(display_width, display_height)
    -- layerPass = layer:getPass()
    -- layerPass:setClear(0xff00ff)
    -- layerTexture = layer:getTexture()
    -- layer:setViewport(0, 0, 500, 500)
    -- lovr.headset.setLayers(layer)
    -- layers = lovr.headset.getLayers()
    
    
    terrain_load()

    gun = lovr.graphics.newModel('ruger.glb')
    gun_collider = world:newCollider(0, 1.5, -2)
    gun_collider:setKinematic(false)
    gun_collider:addShape(lovr.physics.newConvexShape(gun))
    gun_collider:setSleepingAllowed(false)
    gun_collider:setTag("pickupable")

    box_collider = world:newBoxCollider(0, .35, -2, .7)
    box_collider:setKinematic(true)

    
    

    local head_transform = mat4(lovr.headset.getPose("head"))
    local head_position = vec3(head_transform:getPosition())

    print("head", head_position)
    

    
    player_height = 1.80
    local collider_radius = player_height / 8
    
    collider_offset = player_height / 2
    
    
    capsule_collider = world:newCapsuleCollider(0, player_height / 2, 0, collider_radius, player_height - 2 * collider_radius)
    capsule_collider:setKinematic(false)
    -- capsule_collider:setOrientation(math.pi, 0, 1, 0)
    capsule_collider:getShape():setOffset(0, 0, 0, -math.pi / 2, 1, 0, 0)
    capsule_collider:setContinuous(true)
    capsule_collider:setDegreesOfFreedom("xyz", "")
    capsule_collider:setTag("character")

    print("capsule length", player_height - 2 * collider_radius)
    print("collider offset", collider_offset)

    for i = 1, 2 do
        hands.colliders[i] = world:newBoxCollider(vec3(0, 2, 0), vec3(0.04, 0.08, 0.08))
        hands.colliders[i]:setKinematic(true)
        hands.colliders[i]:setSensor(true)
        if i == 1 then
            hands.colliders[i]:setTag("leftHand")
        else
            hands.colliders[i]:setTag("rightHand")
        end
        
        -- hands.colliders[i]:setLinearDamping(0.7)
        -- hands.colliders[i]:setAngularDamping(0.9)
        -- hands.colliders[i]:setMass(0.5)
        -- registerCollisionCallback(hands.colliders[i],
        --                           function(collider, world)
        --                               -- store collider that was last touched by hand
        --                               print("touching")
        --                               hands.touching[i] = collider
        --                           end)
    end

    world:setCallbacks({
        -- filter = function(a, b)
        --     return true
        -- end,
        enter = function(a, b, contact)
            -- play sounds, spawn particles, etc.
            -- the collision has not been resolved yet, so the velocity of a and b
            -- is the velocity before the collision, and can be used to estimate the
            -- collision force
            -- print("collide")

            if a:getTag() == "rightHand" and b:getTag() ~= "leftHand" then
                hands.touching[2] = b
            end

            if a:getTag() == "leftHand" and b:getTag() ~= "rightHand" then
                hands.touching[1] = b
            end

            if b:getTag() == "rightHand" and a:getTag() ~= "leftHand" then
                hands.touching[2] = a
            end

            if b:getTag() == "leftHand" and a:getTag() ~= "rightHand" then
                hands.touching[1] = a
            end

            print("enter")

            -- if a:getTag
            print("a tag", a:getTag())
            print("b tag", b:getTag())
        end,
        exit = function(a, b)
            -- a and b have stopped touching!
            if a:getTag() == "leftHand" or b:getTag() == "leftHand" then
                hands.touching[2] = nil
            end
            
            if a:getTag() == "rightHand" or b:getTag() == "rightHand" then
                hands.touching[2] = nil
            end
            
            
            print("exit")
        end,
        contact = function(a, b, contact)
            -- print("contact")
            -- a and b are touching this frame
            -- use sparingly, as this may be called many times per frame
            -- use Contact:setFriction and Contact:setResitution to update
            -- the contact behavior, or Contact:setSurfaceVelocity, for a
            -- conveyor belt effect, or Contact:setEnabled to disable the
            -- collision completely.
        end
    })
    -- world:update(dt, function(world)
    --                      world:computeOverlaps()
    --                      local overlaps = world:overlaps()
    --                      -- print("overlaps", overlaps:length())
    --                      for shapeA, shapeB in overlaps do
    --                          local areColliding = world:collide(shapeA, shapeB)
    --                          if areColliding then
    --                              cbA = collisionCallbacks[shapeA]
    --                              if cbA then cbA(shapeB:getCollider(), world) end
    --                              cbB = collisionCallbacks[shapeB]
    --                              if cbB then cbB(shapeA:getCollider(), world) end
    --                          end
    --                      end
    --                  end)

    
end

function lovr.update(dt)
    local real_head_pose = mat4(lovr.headset.getPose("head"))
    local real_head_orientation = quat(real_head_pose:getOrientation())
    local real_head_position = vec3(real_head_pose:getPosition())

    local collider_shape = capsule_collider:getShape()
    local x, y, z, angle, ax, ay, az = collider_shape:getOffset()
    local horizontal_offset = vec2(real_head_position.x, real_head_position.z)
    collider_shape:setLength(real_head_position.y - 2 * collider_shape:getRadius())
    collider_shape:setOffset(horizontal_offset.x, 0, horizontal_offset.y, angle, ax, ay, az)

    
    
    virtual_head_pose = mat4(capsule_collider:getPose())
    -- virtual_head_pose[13] = virtual_head_pose[13] + horizontal_offset.x
    virtual_head_pose[14] = virtual_head_pose[14] + real_head_position.y / 2
    -- virtual_head_pose[15] = virtual_head_pose[15] + horizontal_offset.y

    -- print("virtual_head_pose[14]", virtual_head_pose[14])
    
    for i, hand in pairs(lovr.headset.getHands()) do
        local real_hand_pose = mat4(lovr.headset.getPose(hand))
        
        local real_hand_position = vec3(real_hand_pose:getPosition())
        local real_hand_orientation = quat(real_hand_pose:getOrientation())
        local real_hand_orientation_inv = quat(real_hand_orientation):conjugate()
        
        local real_hand_orientation_relative = quat(real_hand_orientation * real_head_orientation)
        local real_hand_position_relative = vec3(real_hand_position - real_head_position)
        local real_hand_pose_relative = mat4(real_hand_position_relative, real_hand_orientation_relative)
        
        -- local virtual_hand_position = vec3(vec3(virtual_head_pose:getPosition()) + vec3(real_hand_position_relative))
        -- local virtual_hand_orientation = quat(real_hand_orientation_relative):mul(quat(virtual_head_pose:getOrientation()))
        -- hands.colliders[i]:setPose(virtual_hand_position, virtual_hand_orientation)

        local virtual_hand_pose = virtual_head_pose * real_hand_pose_relative
        hands.colliders[i]:setPose(vec3(virtual_hand_pose:getPosition()), quat(real_hand_position_relative))
        
    end


        -- local virtual_hand_pose_position = vec3(virtual_head_pose:getPosition()) + real_hand_position_relative
        -- local virtual_hand_pose = virtual_head_pose * real_hand_pose_relative
        -- local virtual_hand_pose = mat4(virtual_head_pose):translate(relative_virtual_hand_position)

    
    

    
    -- local offset = head_position.y / 2
    -- print("offset", offset)

    -- local collider_pose = mat4(capsule_collider:getPose())
    -- collider_pose[14] = 0

    -- print("collider_pose", collider_pose:getPosition())
    
    -- for i, hand in pairs(lovr.headset.getHands()) do
    --     local hand_pose = mat4(lovr.headset.getPose(hand))

    --     local adjusted_hand_pose = hand_pose * collider_pose

    --     local hand_pos = vec3(adjusted_hand_pose:getPosition())
    --     local ref_pose = vec3(collider_pose:getPosition())
        
    --     -- print("hand", hand_pose)
        
    --     -- local hand_position = vec3(hand_pose:getPosition()):rotate(quat(collider_pose))
    --     -- hand_pose:tran(collider_pose)
    --     -- print("hand: " .. i, hand_position.y)
        
        
    --     hands.colliders[i]:setPose(vec3(hand_pos.x + ref_pose.x, hand_pos.y, hand_pos.z + ref_pose.z), quat(adjusted_hand_pose:getOrientation()))

        
    -- --     local rw =  -- real world pose of controllers
    -- --     local vr = mat4(hands.colliders[i]:getPose()) -- vr pose of palm colliders
        
    -- end
    -- -- 

    -- local pose = mat4(capsule_collider:getPose())
    -- pose[14] = pose[14] - collider_offset

    
    -- CapsuleShape:setLength

    
    -- print("collider", motion.pose:getPosition())
    -- capsule_collider:setPosition(vec3(motion.pose:getPosition()) + vec3(0,collider_offset,-1))
    -- capsule_collider:setOrientation(quat(motion.pose:getOrientation()))
    -- capsule_collider:setOrientation(motion.pose:getOrientation())
    -- capsule_collider:setPose(motion.pose:getPosition() + vec3(0, collider_offset, 0), motion.pose:getOrientation())

    -- print(mat4(lovr.headset.getPose()):getScale())
    
    terrain_update(dt)
    locomotion(dt)
    -- motion.update(dt)
    ui_update(dt)


    -- if lovr.headset.wasPressed('right', 'a') then
    --     capsule_collider:applyLinearImpulse(0, 1000, 0)
    -- end

    
    world:update(dt)

    
    
    
    
    -- -- hand updates - location, orientation, solidify on trigger button, grab on grip button
    -- for i, hand in pairs(lovr.headset.getHands()) do
    --     -- align collider with controller by applying force (position) and torque (orientation)
    --     local rw = mat4(lovr.headset.getPose(hand)) -- real world pose of controllers
    --     local vr = mat4(hands.colliders[i]:getPose()) -- vr pose of palm colliders

    --     local collider_pose = mat4(capsule_collider:getPose())
    --     collider_pose[13] = collider_pose[13] + rw[13]
    --     collider_pose[14] = collider_pose[14] + rw[14] - collider_offset
    --     collider_pose[15] = collider_pose[15] + rw[15]
    --     hands.colliders[i]:setPose(vec3(collider_pose:getPosition()), quat(rw:getOrientation()))

    --     hands.solid[i] = lovr.headset.isDown(hand, 'trigger')

    --     -- pose[14] = pose[14] - collider_offset
    --     -- pass:transform(pose:invert())


    --     -- local angle, ax,ay,az = quat(rw):mul(quat(vr):conjugate()):unpack()
    --     -- angle = ((angle + math.pi) % (2 * math.pi) - math.pi) -- for minimal motion wrap to (-pi, +pi) range
    --     -- -- hands.colliders[i]:applyTorque(vec3(ax, ay, az):mul(angle * dt * hand_torque))
    --     -- -- hands.colliders[i]:applyForce((vec3(rw) - vec3(vr)):mul(dt * hand_force))
    --     -- -- solidify when trigger touched
    --     -- hands.solid[i] = lovr.headset.isDown(hand, 'trigger')

    --     -- -- local shape =
    --     -- -- shape[1]:setSensor(not hands.solid[i])
    --     -- -- print()

    --     -- -- hands.colliders[i]:getShapes()[1]:setSensor()
    --     hands.colliders[i]:setSensor(not hands.solid[i])
    --     -- -- hold/release colliders
    --     if lovr.headset.isDown(hand, 'grip') and hands.touching[i] and not hands.holding[i] then

    --         print("grab")

    --         print(hands.colliders[i]:getPosition())
    --         print(hands.colliders[i]:getOrientation())
            
    --         hands.holding[i] = hands.touching[i]
            
    --         -- hands.holding[i]:setKinematic(true)
    --         -- grab object with ball joint to drag it, and slider joint to also match the orientation
    --         lovr.physics.newWeldJoint(hands.colliders[i], hands.holding[i])
    --         -- lovr.physics.newBallJoint(hands.colliders[i], hands.holding[i], vr:mul(0, 0, 0))
    --         -- lovr.physics.newSliderJoint(hands.colliders[i], hands.holding[i], quat(vr):direction())
    --     end
    --     if lovr.headset.wasReleased(hand, 'grip') and hands.holding[i] then
    --         print("release")
    --         for _, joint in ipairs(hands.colliders[i]:getJoints()) do
    --             joint:destroy()
    --         end
    --         -- hands.holding[i]:setKinematic(false)
    --         hands.holding[i] = nil
    --     end

    --     if lovr.headset.wasPressed('right', 'b') and hands.holding[i] ~= nil then
            
    --         gunshot:play()
    --     end
        
    --     if hands.holding[i] then
    --         hands.holding[i]:setPose(
    --             vec3(hands.colliders[i]:getPosition()),
    --             quat(hands.colliders[i]:getOrientation())
    --         )
    --     end
        
    -- end


    
    -- hands.touching = {nil, nil} -- to be set again in collision resolver
end

function lovr.draw(pass)
    
    -- -- Render hands
    -- pass:setColor(1, 1, 1)
    -- local radius = 0.05
    -- for _, hand in ipairs(lovr.headset.getHands()) do
    --     -- Whenever pose of hand or head is used, need to account for VR movement
    --     local poseRW = mat4(lovr.headset.getPose(hand))
    --     local poseVR = mat4(motion.pose):mul(poseRW)
    --     poseVR:scale(radius)
    --     pass:sphere(poseVR)
    -- end



    
    -- pass:setMaterial(camera)
    -- pass:plane(1, 1, -5, 1, 1)
    
    ui_draw(pass)

    
    
    local head_transform = mat4(lovr.headset.getPose("head"))
    local head_position = vec3(head_transform:getPosition())
    local world_position = vec3(motion.pose:getPosition())
    local collider_position = vec3(capsule_collider:getPose())
    local hand_transform = mat4(lovr.headset.getPose("hand/left"))
    local hand_position = vec3(hand_transform:getPosition())
    local hand_collider_position = vec3(mat4(hands.colliders[1]:getPose()):getPosition())
    
    pass:text(
        string.format(
            "Head Position : \nx: %.2f\ny: %.2f\nz: %.2f\n\nCollider Position: \nx: %.2f\ny: %.2f\nz: %.2f",
            head_position.x,
            head_position.y,
            head_position.z,
            collider_position.x,
            collider_position.y,
            collider_position.z
        ),
        vec3(head_transform * vec3(-0.1, 0, -0.7)), .02, head_transform:getOrientation())

    pass:text(
        string.format(
            "Hand Position : \nx: %.2f\ny: %.2f\nz: %.2f\n\nHand Collider Position: \nx: %.2f\ny: %.2f\nz: %.2f",
            hand_position.x,
            hand_position.y,
            hand_position.z,
            hand_collider_position.x,
            hand_collider_position.y,
            hand_collider_position.z
        ),
        vec3(head_transform * vec3(0.1, 0, -0.7)), .02, head_transform:getOrientation())

    


    -- -- pass:setProjection(1, mat4():orthographic(100, 100))
    -- -- pass:setViewPose(1, mat4())

    
    -- -- print(mat4(pass:getViewPose(1)):getPosition())
    -- -- print("views", views)

    
    -- -- local pose_eye_left = mat4(pass:getViewPose(1))
    -- -- local pose_eye_right = mat4(pass:getViewPose(2))

    -- -- local eye_diff = vec3(pose_eye_right:getPosition()).x - vec3(pose_eye_left:getPosition()).x

    -- -- local collider_pose = mat4(capsule_collider:getPose())
    -- -- collider_pose:translate(0, collider_offset, 0)
    -- -- pass:setViewPose(1, mat4(collider_pose):translate(-eye_diff / 2, collider_offset, 0), false)
    -- -- pass:setViewPose(2, mat4(collider_pose):translate(eye_diff / 2, collider_offset, 0), false)
    -- -- pass:setViewPose(2, pose, false)
    

    
    -- -- print("eye diff", eye_diff)
    -- -- print("left", vec3(pose_left:getPosition()).x)
    -- -- print("right", vec3(pose_right:getPosition()).x)
    
    -- -- display world positioned items after this
    


    
    -- -- local pos = vec3(pose:getPosition())

    -- -- pose[13] = pos.x
    -- -- pose[14] = pos.y - collider_offset
    -- -- pose[15] = pos.z

    
    -- -- -- pose:setPosition(pos.x, pos.y + collider_offset, pos.z)
    -- -- -- pose:translate(vec3(0,pos.y + collider_offset,0))

    
    -- local pose = mat4(capsule_collider:getPose())
    -- pose[14] = pose[14] - collider_offset


    -- print("virtual_head_pose", vec3(virtual_head_pose:getPosition()).y)

    local virtual_head_position = vec3(virtual_head_pose:getPosition())
    -- pass:origin()
    pass:rotate(quat(virtual_head_pose:getOrientation()):conjugate())
    pass:translate(vec3(-virtual_head_position.x, -(virtual_head_position.y - head_position.y), -virtual_head_position.z))
    
    -- pass:transform(vec3(-pos.x, 0, -pos.z), vec3(1), quat(rotation):conjugate())
    -- pass:transform(virtual_head_pose:invert())

    gizmo_draw(pass, mat4(capsule_collider:getPose()), 2)

    
    for i, collider in ipairs(hands.colliders) do

        gizmo_draw(pass, mat4(collider:getPose()))
        -- pass:setColor(0.75, 0.56, 0.44)
        -- drawBoxCollider(pass, collider, not hands.solid[i])
    end
    
    pass:setColor(0.925, 0.745, 0.137)

    local box_pose = mat4(box_collider:getPose())
    
    pass:box(vec3(box_pose:getPosition()), vec3(.7), box_pose:getOrientation())
    pass:setColor(0.925, 0, 0)
    pass:draw(gun, mat4(gun_collider:getPose()))
    gizmo_draw(pass, mat4(gun_collider:getPose()), 1)

    
    -- -- pass:setViewport(0, 0, 100, 100)
    -- -- pass:fill(camera)

    
    -- -- print("gun_collider", gun_collider:getPose())

    pass:setWireframe(true)
    phywire.draw(pass, world)
    
    terrain_draw(pass)
    
    -- -- pass:setColor(.1, .1, .12)
    -- -- pass:plane(0, 0, 0, 100, 100, -math.pi / 2, 1, 0, 0)
    -- -- pass:setColor(.2, .2, .2)
    -- -- pass:plane(0, 1e-5, 0, 100, 100, -math.pi / 2, 1, 0, 0, 'line', 100, 100)

    -- -- for i, hand in ipairs(lovr.headset.getHands()) do
    -- --     hand = hand .. '/point'
    -- --     local position = vec3(lovr.headset.getPosition(hand))
    -- --     local direction = quat(lovr.headset.getOrientation(hand)):direction()

    -- --     pass:setColor(1, 1, 1)
    -- --     pass:sphere(position, .01)

    -- --     pass:setColor(1, 0, 0)
    -- --     pass:line(position, position + direction * 50)
    -- -- end
    

    
    -- -- pass:setViewport(100, 100, 250, 250)
    -- -- pass:setViewPose(1, mat4():lookAt(vec3(0,2,-10), vec3(capsule_collider:getPosition())), true)
    -- -- pass:setViewPose(2, mat4():lookAt(vec3(0,2,-10), vec3(capsule_collider:getPosition())), true)

    -- -- pass:setViewport(0, 0, 100, 100)
    -- -- pass:fill(camera)

    -- pass:origin()



    -- pass:setColor(1,1,1)
    -- layerPass:reset()
    -- layerPass:setColor(0, 0, 1)
    -- layerPass:box(0, 2, -2, 2)
    -- pass:setMaterial(layerTexture)
    
    -- pass:setMaterial(camera)
    -- pass:plane(mat4(lovr.headset.getPose()):translate(0, 0, -2):scale(.5, .5))
    -- pass:plane()

    -- pass:origin()
    -- cameraPass:reset()
    -- camera_draw(cameraPass)
    -- pass:setViewport(0, 0, 100, 100)
    -- pass:fill(camera)


    
    return lovr.graphics.submit(pass)
end

function camera_draw(pass)
    pass:setWireframe(false)
    pass:setColor(0, 0, 1)
    pass:box(0, 2, -10, 50)
end

function gizmo_draw(pass, pose, length)
    pass:push('state')
    local origin = vec3(pose:getPosition())
    length = length or 0.5
    pass:setColor(1,0,0)
    pass:line(origin, vec3(length, 0, 0):transform(pose))
    pass:setColor(0, 1, 0)
    pass:line(origin, vec3(0, length, 0):transform(pose))
    pass:setColor(0, 0.5, 3)
    pass:line(origin, vec3(0, 0, length):transform(pose))
    -- pass:origin()
    pass:pop('state')
end

function terrain_load()
    size = 50
    
    local vertices, indices = grid(size, 100)
    for vi = 1, #vertices do
        local x, y, z = unpack(vertices[vi])
        vertices[vi][2] = terrain_fn(x, z) -- elevate grid to terrain height
    end
    mesh = lovr.graphics.newMesh(vertices)
    world:newTerrainCollider(size, terrain_fn) -- use callback to define elevations
end

function terrain_update(dt)
    -- if lovr.timer.getTime() % 1 < dt then -- spawn new box each second
    --     local collider = world:newBoxCollider(
    --         lovr.math.randomNormal(size / 10, 0),
    --         lovr.math.randomNormal(1, 20),
    --         lovr.math.randomNormal(size / 10, 0),
    --         1)
    --     table.insert(box_colliders, collider)
    -- end
end

function terrain_draw(pass)
    pass:setWireframe(true)
    pass:setColor(0.925, 0.745, 0.137)

    pass:setColor(0.565, 0.404, 0.463)
    pass:draw(mesh)

    pass:setColor(0.388, 0.302, 0.412, 0.1)
    pass:draw(mesh)
end

function grid(size, subdivisions)
    local vertices = {}
    local indices  = {}
    local step     = size / (subdivisions - 1)
    for z = -size / 2, size / 2, step do
        for x = -size / 2, size / 2, step do
            table.insert(vertices, { x, 0, z })
            table.insert(vertices, { x, 0, z + step })
            table.insert(vertices, { x + step, 0, z })
            table.insert(vertices, { x, 0, z + step })
            table.insert(vertices, { x + step, 0, z + step })
            table.insert(vertices, { x + step, 0, z })
        end
    end
    return vertices
end

function terrain_fn(x, z)
    return 0
    -- return 4 * (lovr.math.noise(x * 0.05, z * 0.05) - 0.5)
end

function locomotion(dt)
    if lovr.headset.isTracked('left') then
        local origin = vec3(capsule_collider:getPosition())
        local endpoint = origin - vec3(0, 0.7, 0)
        local collider = world:raycast(origin, endpoint, "~character")
        local onGround = collider and true

        local x, y = lovr.headset.getAxis('left', 'thumbstick')

        local velocity = vec3()

        if math.abs(x) > motion.thumbstickDeadzone then
            velocity.x = x
        end
        
        if math.abs(y) > motion.thumbstickDeadzone then
            velocity.z = -y
        end

        if velocity.x ~= 0 or velocity.z ~= 0 then
            velocity:rotate(quat(lovr.headset.getOrientation("head")))
            velocity:mul(lovr.headset.isDown("left", "thumbstick") and vrController.runningSpeed or vrController.walkingSpeed)
            velocity.y = 0

            capsule_collider:setLinearVelocity(velocity)
        end

        if onGround and lovr.headset.wasPressed('right', 'a') then
            capsule_collider:applyLinearImpulse(0, 1000, 0)
        end
    end
end

function motion.smooth(dt)
    if lovr.headset.isTracked('right') then
        local x, y = lovr.headset.getAxis('right', 'thumbstick')
        -- Smooth horizontal turning
        if math.abs(x) > motion.thumbstickDeadzone then
            -- local hx, _, hz = lovr.headset.getPosition()
            -- motion.pose:translate(hx, 0, hz)
            -- motion.pose:rotate(-x * motion.turningSpeed * dt, 0, 1, 0)
            -- motion.pose:translate(-hx, 0, -hz)

            local pose = mat4(capsule_collider:getPose())
            local hx, _, hz = pose:getPosition()
            -- pose:translate(hx, 0, hz)
            pose:rotate(-x * motion.turningSpeed * dt, 0, 1, 0)
            -- pose:translate(-hx, 0, -hz)
            capsule_collider:setPose(vec3(pose:getPosition()), quat(pose:getOrientation()))
            
        end
    end
    if lovr.headset.isTracked('left') then
        local x, y = lovr.headset.getAxis('left', 'thumbstick')
        local direction = quat(lovr.headset.getOrientation(motion.directionFrom)):direction()
        if not motion.flying then
            direction.y = 0
        end
        -- Smooth strafe movement
        local right_vector = vec3()
        local forward_vector = vec3()

        if math.abs(y) > motion.thumbstickDeadzone then
            forward_vector = direction * y
        end
        
        if math.abs(x) > motion.thumbstickDeadzone then
            right_vector = quat(-math.pi / 2, 0, 1, 0):mul(vec3(direction))
            right_vector:mul(x)
        end

        -- motion.pose:translate(moveVector)

        local moveVector = (forward_vector + right_vector) * motion.walkingSpeed * dt
        local pose = mat4(capsule_collider:getPose())
        pose:translate(moveVector)
        capsule_collider:setPose(vec3(pose:getPosition()), quat(pose:getOrientation()))
        

        -- if math.abs(y) > motion.thumbstickDeadzone and motion.thumbstickCooldown < 0 then
        --     direction:mul(y / math.abs(y) * motion.dashDistance)
        --     motion.pose:translate(direction)
        --     local pose = mat4(capsule_collider:getPose())
        --     pose:translate(direction)
        --     capsule_collider:setPose(vec3(pose:getPosition()), quat(pose:getOrientation()))
        --     motion.thumbstickCooldown = motion.thumbstickCooldownTime
        -- end


        
        -- local moveVector = quat(lovr.headset.getOrientation('head')):direction()
        
        -- if math.abs(x) > motion.thumbstickDeadzone then
        --     moveVector:mul(y / math.abs(y) * motion.dashDistance)

            
        --     local strafeVector = quat(-math.pi / 2, 0, 1, 0):mul(vec3(direction))
        --     motion.pose:translate(strafeVector * x * motion.walkingSpeed * dt)
        -- end
        -- -- Smooth Forward/backward movement
        -- if math.abs(y) > motion.thumbstickDeadzone then
        --     motion.pose:translate(direction * y * motion.walkingSpeed * dt)
        -- end

        -- motion.pose:translate(moveVector * motion.walkingSpeed * dt)
        
        -- local pose = mat4(capsule_collider:getPose())
        -- pose:translate(moveVector)
        -- capsule_collider:setPose(vec3(pose:getPosition()), quat(pose:getOrientation()))

        
    end
end

function motion.snap(dt)
    -- Snap horizontal turning
    if lovr.headset.isTracked('right') then
        local x, y = lovr.headset.getAxis('right', 'thumbstick')
        if math.abs(x) > motion.thumbstickDeadzone and motion.thumbstickCooldown < 0 then
            local angle = -x / math.abs(x) * motion.snapTurnAngle
            local hx, _, hz = lovr.headset.getPosition()
            -- motion.pose:translate(hx, 0, hz)
            -- motion.pose:rotate(angle, 0, 1, 0)
            -- motion.pose:translate(-hx, 0, -hz)
            motion.thumbstickCooldown = motion.thumbstickCooldownTime
        end
    end
    -- Dashing forward/backward
    if lovr.headset.isTracked('left') then
        local x, y = lovr.headset.getAxis('left', 'thumbstick')
        if math.abs(y) > motion.thumbstickDeadzone and motion.thumbstickCooldown < 0 then
            local moveVector = quat(lovr.headset.getOrientation('head')):direction()
            if not motion.flying then
                moveVector.y = 0
            end
            moveVector:mul(y / math.abs(y) * motion.dashDistance)
            -- motion.pose:translate(moveVector)
            local pose = mat4(capsule_collider:getPose())
            pose:translate(moveVector)
            capsule_collider:setPose(vec3(pose:getPosition()), quat(pose:getOrientation()))
            motion.thumbstickCooldown = motion.thumbstickCooldownTime
        end
    end
    motion.thumbstickCooldown = motion.thumbstickCooldown - dt
end

function motion.update(dt)
    motion.directionFrom = lovr.headset.isDown('left', 'trigger') and 'left' or 'head'
    if lovr.headset.isDown('left', 'grip') then
        motion.flying = true
    elseif lovr.headset.wasReleased('left', 'grip') then
        motion.flying = false
        -- local height = vec3(motion.pose).y
        -- motion.pose:translate(0, -height, 0)
    end
    -- if lovr.headset.isDown('right', 'grip') then
    --     motion.snap(dt)
    -- else
        motion.smooth(dt)
    -- end
end

function raycast(rayPos, rayDir, planePos, planeDir)
  local dot = rayDir:dot(planeDir)
  if math.abs(dot) < .001 then
    return nil
  else
    local distance = (planePos - rayPos):dot(planeDir) / dot
    if distance > 0 then
      return rayPos + rayDir * distance
    else
      return nil
    end
  end
end

function ui_draw(pass)
    -- pass:push('state')
    -- Button background
    if button.active then
        pass:setColor(.4, .4, .4)
    elseif button.hover then
        pass:setColor(.2, .2, .2)
    else
        pass:setColor(.1, .1, .1)
    end
    pass:plane(button.position, button.width, button.height)

    -- Button text (add a small amount to the z to put the text slightly in front of button)
    pass:setColor(1, 1, 1)
    pass:text(button.text, button.position + vec3(0, 0, .001), button.textSize)
    pass:text('Count: ' .. button.count, button.position + vec3(0, .5, 0), .1)

    -- Pointers
    -- for hand, tip in pairs(tips) do
    --     local position = vec3(lovr.headset.getPosition(hand))

    --     pass:setColor(1, 1, 1)
    --     pass:sphere(position, .01)

    --     if button.active then
    --         pass:setColor(0, 1, 0)
    --     else
    --         pass:setColor(1, 0, 0)
    --     end

    --     pass:line(position, tip)
    -- end
    -- pass:pop('state')
end

function ui_update(dt)
    button.hover, button.active = false, false


    -- https://lovr.org/docs/Device
    -- local gripPose = vec3(lovr.headset.getPosition("elbow/left"))
    -- print("elbowLeft", elbowLeft)
    
    
    for i, hand in ipairs(lovr.headset.getHands()) do
        tips[hand] = tips[hand] or lovr.math.newVec3()

        -- Ray info:
        local rayPosition = vec3(lovr.headset.getPosition(hand .. '/point'))
        local rayDirection = vec3(lovr.headset.getDirection(hand .. '/point'))

        -- Call the raycast helper function to get the intersection point of the ray and the button plane
        local hit = raycast(rayPosition, rayDirection, button.position, vec3(0, 0, 1))

        local inside = false
        if hit then
            local bx, by, bw, bh = button.position.x, button.position.y, button.width / 2, button.height / 2
            inside = (hit.x > bx - bw) and (hit.x < bx + bw) and (hit.y > by - bh) and (hit.y < by + bh)
        end

        -- If the ray intersects the plane, do a bounds test to make sure the x/y position of the hit
        -- is inside the button, then mark the button as hover/active based on the trigger state.
        if inside then
            if lovr.headset.isDown(hand, 'trigger') then
                button.active = true
            else
                button.hover = true
            end

            if lovr.headset.wasReleased(hand, 'trigger') then
                button.count = button.count + 1
                lovr.event.restart()
            end
        end

        -- Set the end position of the pointer.  If the raycast produced a hit position then use that,
        -- otherwise extend the pointer's ray outwards by 50 meters and use it as the tip.
        tips[hand]:set(inside and hit or (rayPosition + rayDirection * 50))
    end
end

function drawBoxCollider(pass, collider, is_sensor)
  -- query current pose (location and orientation)
    local pose = mat4(collider:getPose())
    gizmo_draw(pass, pose)
  -- query dimensions of box
  local shape = collider:getShapes()[1]
  local size = vec3(shape:getDimensions())
  -- draw box
  pose:scale(size)
  pass:box(pose, is_sensor and 'line' or 'fill')
end


function registerCollisionCallback(collider, callback)
  collisionCallbacks = collisionCallbacks or {}
  for _, shape in ipairs(collider:getShapes()) do
    collisionCallbacks[shape] = callback
  end
  -- to be called with arguments callback(otherCollider, world) from update function
end
