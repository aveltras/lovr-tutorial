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

local tips = {}


local hands = {                   -- palms that can push and grab objects
    colliders = { nil, nil },     -- physical objects for palms
    touching  = { nil, nil },     -- the collider currently touched by each hand
    holding   = { nil, nil },     -- the collider attached to palm
    solid     = { false, false }, -- hand can either pass through objects or be solid
}

local hand_torque = 10
local hand_force = 300

local wireframeEnabled = false
local phywireEnabled = false

local pickupable = {}
pickupable.__index = pickupable

function pickupable:new(model, userData)
    model = lovr.graphics.newModel('ruger.glb')
    collider = world:newCollider(0, 1.5, -2)
    collider:setKinematic(false)
    collider:addShape(lovr.physics.newConvexShape(model))
    collider:setSleepingAllowed(false)
    collider:setTag("pickupable")
    collider:setUserData(userData)
end

local vrController = {}
vrController.__index = vrController

function vrController:new(world, playerHeight)
    local collider_radius = playerHeight / 8
    local headRadius = collider_radius / 2

    bodyCollider = world:newCollider(0, 0, 0)
    bodyCollider:setKinematic(false)
    bodyCollider:setContinuous(true)
    bodyCollider:setDegreesOfFreedom("xyz", "")
    bodyCollider:setTag("character")
    bodyCollider:setLinearDamping(3)

    headShape = lovr.physics.newSphereShape(headRadius)
    headShape:setOffset(0, playerHeight, 0)
    bodyCollider:addShape(headShape)

    bodyShape = lovr.physics.newCapsuleShape(collider_radius, playerHeight - 2 * collider_radius)
    bodyShape:setOffset(0, playerHeight / 2, 0, -math.pi / 2, 1, 0, 0)
    bodyCollider:addShape(bodyShape)

    local handColliders = { nil, nil }

    for i = 1, 2 do
        handColliders[i] = world:newBoxCollider(vec3(0, 2, 0), vec3(0.04, 0.08, 0.08))
        handColliders[i]:setKinematic(true)
        handColliders[i]:setSensor(true)
        handColliders[i]:setTag("hand")
    end

    local self = {
        world = world,
        origin = lovr.math.newMat4(),
        playerHeight = playerHeight,
        bodyCollider = bodyCollider,
        handColliders = handColliders,
        handVelocity = { lovr.math.newVec3(), lovr.math.newVec3() },
        itemHeld = { nil, nil },
        thumbstickDeadzone = 0.4,
        turnMode = "smooth",
        walkingSpeed = 6,
        runningSpeed = 10,
        smoothTurnSpeed = 1,
        snapTurnAngle = math.pi / 6,
        snapTurnCooldownDuration = 0.2,
        snapTurnCooldown = 0,
    }

    return setmetatable(self, vrController)
end

function vrController:getHeadPose()
    return self.bodyCollider:getShapes()[2]:getPose()
end

function vrController:update(dt)
    local previousOrigin = mat4(self.origin);
    self.origin:set(self.bodyCollider:getPose())

    -- local rawVelocity = (vec3(self.origin:getPosition()) - vec3(previousOrigin:getPosition())) / dt
    -- print("rawVelocity", rawVelocity:length())

    local realHeadPose = mat4(lovr.headset.getPose("head"))
    local realHeadOrientation = quat(realHeadPose:getOrientation())
    local realHeadPosition = vec3(realHeadPose:getPosition())

    local shapes = self.bodyCollider:getShapes()

    -- body
    shapes[1]:setOffset(0, realHeadPosition.y / 2, 0, -math.pi / 2, 1, 0, 0)
    shapes[1]:setLength(realHeadPosition.y - 2 * shapes[1]:getRadius())

    -- head
    shapes[2]:setOffset(realHeadPosition.x, realHeadPosition.y, realHeadPosition.z)

    if lovr.headset.isTracked('right') then
        local x, y = lovr.headset.getAxis('right', 'thumbstick')
        if math.abs(x) > self.thumbstickDeadzone then
            if self.turnMode == "snap" then -- Snap horizontal turning
                if self.snapTurnCooldown > 0 then
                    self.snapTurnCooldown = self.snapTurnCooldown - dt
                else
                    local pose = mat4(self.bodyCollider:getPose())
                    local angle = -x / math.abs(x) * self.snapTurnAngle
                    local hx, _, hz = lovr.headset.getPosition()
                    pose:translate(hx, 0, hz)
                    pose:rotate(angle, 0, 1, 0)
                    pose:translate(-hx, 0, -hz)
                    self.bodyCollider:setPose(vec3(pose:getPosition()), quat(pose:getOrientation()))
                    self.snapTurnCooldown = self.snapTurnCooldownDuration
                end
            else -- Smooth horizontal turning
                local pose = mat4(self.bodyCollider:getPose())
                local hx, _, hz = lovr.headset.getPosition()
                pose:translate(hx, 0, hz)
                pose:rotate(-x * self.smoothTurnSpeed * dt, 0, 1, 0)
                pose:translate(-hx, 0, -hz)
                self.bodyCollider:setPose(vec3(pose:getPosition()), quat(pose:getOrientation()))
            end
        end
    end

    if lovr.headset.isTracked('left') then
        local origin = vec3(self.bodyCollider:getPosition())
        local endpoint = origin - vec3(0, 0.10, 0)
        local collider = world:raycast(origin + vec3(0, 0.05, 0), endpoint, "~character")
        local onGround = collider and true

        local x, y = lovr.headset.getAxis('left', 'thumbstick')

        local velocity = vec3()

        if math.abs(x) > self.thumbstickDeadzone then
            velocity.x = x
        end

        if math.abs(y) > self.thumbstickDeadzone then
            velocity.z = -y
        end

        if velocity.x ~= 0 or velocity.z ~= 0 then
            velocity:rotate(self.bodyCollider:getOrientation())
            velocity:rotate(quat(lovr.headset.getOrientation("head")))
            velocity:mul(lovr.headset.isDown("left", "thumbstick") and self.runningSpeed or self.walkingSpeed)
            velocity.y = 0

            self.bodyCollider:applyForce(velocity * 500)
        end

        if onGround and lovr.headset.wasPressed('right', 'a') then
            self.bodyCollider:applyLinearImpulse(0, 1000, 0)
        end
    end

    for i, hand in pairs(lovr.headset.getHands()) do
        local realHandPose = mat4(lovr.headset.getPose(hand))
        local realHandPosition = vec3(realHandPose:getPosition())
        local realHandOrientation = quat(realHandPose:getOrientation())
        local handPositionRelativeToHead = vec3(realHandPosition - realHeadPosition)
        local adjustedHandPose = mat4(headShape:getPose()):translate(handPositionRelativeToHead):rotate(
        realHandOrientation)

        self.handColliders[i]:setPose(vec3(adjustedHandPose:getPosition()), quat(adjustedHandPose:getOrientation()))

        if self.itemHeld[i] then
            if lovr.headset.wasReleased(hand, 'grip') then
                self.itemHeld[i]:destroy()
                self.itemHeld[i] = nil
            elseif lovr.headset.wasPressed(hand, 'trigger') then
                local _, itemCollider = self.itemHeld[i]:getColliders()
                local userData = itemCollider:getUserData()
                if userData and userData.onTriggerPressed then
                    userData.onTriggerPressed()
                end
            end
        elseif lovr.headset.wasPressed(hand, 'grip') then
            local shape = self.handColliders[i]:getShape()
            local collider = self.world:overlapShape(
                shape,
                vec3(shape:getPosition()),
                quat(shape:getOrientation()),
                0,
                "pickupable"
            )

            if collider then
                collider:setPose(
                    vec3(self.handColliders[i]:getPosition()),
                    quat(self.handColliders[i]:getOrientation()):mul(quat(-math.pi / 2, 1, 0, 0))
                )
                self.itemHeld[i] = lovr.physics.newWeldJoint(self.handColliders[i], collider)
            end
        end
    end
end

function vrController:draw(pass)
    for i, collider in ipairs(self.handColliders) do
        gizmo_draw(pass, mat4(collider:getPose()), 0.2)
    end
end

function lovr.load()
    lovr.graphics.setBackgroundColor(0.208, 0.208, 0.275)
    world = lovr.physics.newWorld({
        tags = { "character", "hand", "leftHand", "rightHand", "pickupable" }
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

    -- pickupable:new

    gun = lovr.graphics.newModel('ruger.glb')
    gun_collider = world:newCollider(0, 1.5, -2)
    gun_collider:setKinematic(false)
    gun_collider:addShape(lovr.physics.newConvexShape(gun))
    gun_collider:setSleepingAllowed(false)
    gun_collider:setTag("pickupable")
    gun_collider:setUserData({
        onTriggerPressed = function(a, b, contact)
            gunshot:play()
        end
    })


    box_collider = world:newBoxCollider(0, .35, -2, .7)
    box_collider:setKinematic(true)


    wall = world:newBoxCollider(0, 1, -5, 10, 2, .2)
    wall:setKinematic(true)


    local head_transform = mat4(lovr.headset.getPose("head"))
    local head_position = vec3(head_transform:getPosition())

    print("head", head_position)

    controller = vrController:new(world, 1.80)

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

    -- world:setCallbacks({
    --     -- filter = function(a, b)
    --     --     return true
    --     -- end,
    --     enter = function(a, b, contact)
    --         -- play sounds, spawn particles, etc.
    --         -- the collision has not been resolved yet, so the velocity of a and b
    --         -- is the velocity before the collision, and can be used to estimate the
    --         -- collision force
    --         -- print("collide")

    --         if a:getTag() == "rightHand" and b:getTag() ~= "leftHand" then
    --             hands.touching[2] = b
    --         end

    --         if a:getTag() == "leftHand" and b:getTag() ~= "rightHand" then
    --             hands.touching[1] = b
    --         end

    --         if b:getTag() == "rightHand" and a:getTag() ~= "leftHand" then
    --             hands.touching[2] = a
    --         end

    --         if b:getTag() == "leftHand" and a:getTag() ~= "rightHand" then
    --             hands.touching[1] = a
    --         end

    --         print("enter")

    --         -- if a:getTag
    --         print("a tag", a:getTag())
    --         print("b tag", b:getTag())
    --     end,
    --     exit = function(a, b)
    --         -- a and b have stopped touching!
    --         if a:getTag() == "leftHand" or b:getTag() == "leftHand" then
    --             hands.touching[2] = nil
    --         end

    --         if a:getTag() == "rightHand" or b:getTag() == "rightHand" then
    --             hands.touching[2] = nil
    --         end


    --         print("exit")
    --     end,
    --     contact = function(a, b, contact)
    --         -- print("contact")
    --         -- a and b are touching this frame
    --         -- use sparingly, as this may be called many times per frame
    --         -- use Contact:setFriction and Contact:setResitution to update
    --         -- the contact behavior, or Contact:setSurfaceVelocity, for a
    --         -- conveyor belt effect, or Contact:setEnabled to disable the
    --         -- collision completely.
    --     end
    -- })
end

function lovr.update(dt)
    controller:update(dt)
    terrain_update(dt)
    ui_update(dt)
    world:update(dt)

    if lovr.system.wasKeyReleased('z') then
        print("button")
        wireframeEnabled = not wireframeEnabled
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


    -- locomotion(dt)
    -- motion.update(dt)



    -- if lovr.headset.wasPressed('right', 'a') then
    --     capsule_collider:applyLinearImpulse(0, 1000, 0)
    -- end








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



    -- local world_position = vec3(motion.pose:getPosition())
    -- local collider_position = vec3(capsule_collider:getPose())
    local hand_transform = mat4(lovr.headset.getPose("hand/left"))
    local hand_position = vec3(hand_transform:getPosition())
    local hand_collider_position = vec3(mat4(hands.colliders[1]:getPose()):getPosition())



    -- pass:text(
    --     string.format(
    --         "Head Position : \nx: %.2f\ny: %.2f\nz: %.2f\n\nCollider Position: \nx: %.2f\ny: %.2f\nz: %.2f",
    --         head_position.x,
    --         head_position.y,
    --         head_position.z,
    --         collider_position.x,
    --         collider_position.y,
    --         collider_position.z
    --     ),
    --     vec3(head_transform * vec3(-0.1, 0, -0.7)), .02, head_transform:getOrientation())

    -- pass:text(
    --     string.format(
    --         "Hand Position : \nx: %.2f\ny: %.2f\nz: %.2f\n\nHand Collider Position: \nx: %.2f\ny: %.2f\nz: %.2f",
    --         hand_position.x,
    --         hand_position.y,
    --         hand_position.z,
    --         hand_collider_position.x,
    --         hand_collider_position.y,
    --         hand_collider_position.z
    --     ),
    --     vec3(head_transform * vec3(0.1, 0, -0.7)), .02, head_transform:getOrientation())




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


    -- for i, hand in ipairs(lovr.headset.getHands()) do
    --     -- pass:cone(vec3(lovr.headset.getPosition(hand)), 0.10, 0.2, quat(lovr.headset.getOrientation(hand)))
    --     -- gizmo_draw(pass, mat4(lovr.headset.getPose(hand)), 0.2)
    -- end


    -- -- local pos = vec3(pose:getPosition())

    -- -- pose[13] = pos.x
    -- -- pose[14] = pos.y - collider_offset
    -- -- pose[15] = pos.z


    -- -- -- pose:setPosition(pos.x, pos.y + collider_offset, pos.z)
    -- -- -- pose:translate(vec3(0,pos.y + collider_offset,0))


    -- local pose = mat4(capsule_collider:getPose())
    -- pose[14] = pose[14] - collider_offset


    -- print("virtual_head_pose", vec3(virtual_head_pose:getPosition()).y)

    local virtual_head_pose = mat4(controller:getHeadPose())
    local virtual_head_position = vec3(virtual_head_pose:getPosition())

    -- pass:transform(virtual_head_pose:invert())

    local viewPose = mat4(pass:getViewPose(2))
    local viewPosition = vec3(viewPose:getPosition())


    pass:text(
        string.format(
            "Head Position : \nx: %.2f\ny: %.2f\nz: %.2f\n\nVirtual Head Position : \nx: %.2f\ny: %.2f\nz: %.2f\n\nView Position : \nx: %.2f\ny: %.2f\nz: %.2f",
            head_position.x,
            head_position.y,
            head_position.z,
            virtual_head_position.x,
            virtual_head_position.y,
            virtual_head_position.z,
            viewPosition.x,
            viewPosition.y,
            viewPosition.z
        ),
        vec3(head_transform * vec3(-0.1, 0, -0.7)), .02, head_transform:getOrientation())


    local hx, _, hz = lovr.headset.getPosition()
    pass:translate(hx, 0, hz)
    pass:rotate(quat(virtual_head_pose:getOrientation()):conjugate())
    pass:translate(-hx, 0, -hz)

    pass:translate(
        -(virtual_head_position.x - head_position.x),
        -(virtual_head_position.y - head_position.y),
        -(virtual_head_position.z - head_position.z)
    )

    -- pass:transform(vec3(-pos.x, 0, -pos.z), vec3(1), quat(rotation):conjugate())
    -- pass:transform(virtual_head_pose:invert())

    -- gizmo_draw(pass, mat4(capsule_collider:getPose()), 2)

    controller:draw(pass)

    pass:setColor(0.925, 0.745, 0.137)

    local box_pose = mat4(box_collider:getPose())

    pass:box(vec3(box_pose:getPosition()), vec3(.7), box_pose:getOrientation())
    pass:setColor(0.925, 0, 0)
    pass:draw(gun, mat4(gun_collider:getPose()))
    gizmo_draw(pass, mat4(gun_collider:getPose()), 1)

    -- pass:cone(0, 0, 0, 0.25, 0.5)

    -- -- pass:setViewport(0, 0, 100, 100)
    -- -- pass:fill(camera)


    -- -- print("gun_collider", gun_collider:getPose())


    -- wall = world:newBoxCollider(0, 2, -5, 10, 2, .2)


    local wallPos = vec3(wall:getPosition())
    pass:setColor(0.2, 0.2, 0.2)
    pass:box(wallPos.x, wallPos.y, wallPos.z, 10, 2, .2, wall:getOrientation())


    pass:setWireframe(wireframeEnabled)

    if phywireEnabled then
        phywire.draw(pass, world)
    end

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
    pass:setColor(1, 0, 0)
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
    pass:setWireframe(wireframeEnabled)
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
