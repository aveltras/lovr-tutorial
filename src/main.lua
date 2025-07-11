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

local bullets = {}

local bottles = {}

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
        handPreviousPose = { lovr.math.newMat4(), lovr.math.newMat4() },
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
    -- local previousOrigin = mat4(self.origin);
    self.origin:set(self.bodyCollider:getPose())

    -- local rawVelocity = (vec3(self.origin:getPosition()) - vec3(previousOrigin:getPosition())) / dt
    -- print("rawVelocity", rawVelocity:length())

    local realHeadPose = mat4(lovr.headset.getPose("head"))
    -- local realHeadOrientation = quat(realHeadPose:getOrientation())
    local realHeadPosition = vec3(realHeadPose:getPosition())

    local shapes = self.bodyCollider:getShapes()

    -- body
    shapes[1]:setOffset(0, realHeadPosition.y / 2, 0, -math.pi / 2, 1, 0, 0)
    shapes[1]:setLength(realHeadPosition.y - 2 * shapes[1]:getRadius())

    -- head
    shapes[2]:setOffset(realHeadPosition.x, realHeadPosition.y, realHeadPosition.z)

    if lovr.headset.isTracked('right') then
        local x, _ = lovr.headset.getAxis('right', 'thumbstick')
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

        local previousPosition = vec3(self.handPreviousPose[i]:getPosition())
        local currentPosition = vec3(adjustedHandPose:getPosition())
        local rawVelocity = (currentPosition - previousPosition) / dt
        self.handVelocity[i]:set(self.handVelocity[i]:lerp(rawVelocity, 0.1))
        local predictedPosition = vec3(currentPosition + self.handVelocity[i] * dt)
        self.handPreviousPose[i]:set(adjustedHandPose)
        self.handColliders[i]:setPose(predictedPosition, quat(adjustedHandPose:getOrientation()))

        if self.itemHeld[i] then
            if lovr.headset.wasReleased(hand, 'grip') then
                local _, itemCollider = self.itemHeld[i]:getColliders()
                -- itemCollider:setGravityScale(1)
                -- self.itemHeld[i]:setKinematic(false)
                itemCollider:setLinearVelocity(self.handVelocity[i])
                self.itemHeld[i]:destroy()
                self.itemHeld[i] = nil
            else
                local _, itemCollider = self.itemHeld[i]:getColliders()
                itemCollider:getUserData().update(dt, hand, self.handColliders[i])
                -- self.itemHeld[i]:getUserData().update(dt, hand, self.handColliders[i])
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
                -- collider:setKinematic(true)
                -- self.itemHeld[i] = collider
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
        tags = { "character", "hand", "leftHand", "rightHand", "pickupable" },
        stabilization = 0.8,
    })

    world:disableCollisionBetween("character", "pickupable")

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
    gun_collider:addShape(lovr.physics.newConvexShape(gun))
    gun_collider:setTag("pickupable")
    gun_collider:setUserData({
        update = function(dt, device, holder)
            if lovr.headset.wasPressed(device, 'trigger') then
                local direction = quat(gun_collider:getOrientation()):direction()
                local origin = vec3(gun_collider:getPosition())
                local bulletCollider = world:newSphereCollider(origin, 0.01)
                bulletCollider:setContinuous(true)
                bulletCollider:setMass(0.021)
                bulletCollider:setLinearVelocity(direction * 100)
                bullets[#bullets + 1] = bulletCollider
                local source = gunshot:clone()
                source:play()
            end
        end,
    })

    automatic_gun_collider = world:newCollider(0, 1.5, -2)
    automatic_gun_collider:addShape(lovr.physics.newConvexShape(gun))
    automatic_gun_collider:setTag("pickupable")
    automatic_gun_collider:setUserData({
        cooldown = 0,
        update = function(dt, device, holder)
            if lovr.headset.isDown(device, 'trigger') then
                if automatic_gun_collider:getUserData().cooldown < 0 then
                    local direction = quat(automatic_gun_collider:getOrientation()):direction()
                    local origin = vec3(automatic_gun_collider:getPosition())
                    local bulletCollider = world:newSphereCollider(origin, 0.01)
                    bulletCollider:setContinuous(true)
                    bulletCollider:setMass(0.021)
                    bulletCollider:setLinearVelocity(direction * 100)
                    bullets[#bullets + 1] = bulletCollider
                    local source = gunshot:clone()
                    source:play()
                    automatic_gun_collider:getUserData().cooldown = 0.1
                end
            end

            automatic_gun_collider:getUserData().cooldown = automatic_gun_collider:getUserData().cooldown - dt
        end,
    })


    box_collider = world:newBoxCollider(0, .35, -2, .7)
    box_collider:setKinematic(true)


    wall = world:newBoxCollider(0, 1, -5, 10, 2, .2)
    wall:setKinematic(true)

    for offset = -2, 2, 0.50 do
        local bottle = world:newCylinderCollider(offset, 2.25, -5, 0.05, 0.5)
        bottle:setOrientation(-math.pi / 2, 1, 0, 0)
        table.insert(bottles, bottle)
    end

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
end

function lovr.draw(pass)
    ui_draw(pass)
    local head_transform = mat4(lovr.headset.getPose("head"))
    local head_position = vec3(head_transform:getPosition())
    -- local world_position = vec3(motion.pose:getPosition())
    -- local collider_position = vec3(capsule_collider:getPose())
    local hand_transform = mat4(lovr.headset.getPose("hand/left"))
    local hand_position = vec3(hand_transform:getPosition())
    local hand_collider_position = vec3(mat4(hands.colliders[1]:getPose()):getPosition())


    for i, hand in pairs(lovr.headset.getHands()) do
        gizmo_draw(pass, mat4(lovr.headset.getPose(hand)), 0.2)
    end
    --

    local virtual_head_pose = mat4(controller:getHeadPose())
    local virtual_head_position = vec3(virtual_head_pose:getPosition())

    pass:text(
        string.format(
            [[
              Head Position:
              x: %.2f
              y: %.2f
              z: %.2f

              Virtual Head Position:
              x: %.2f
              y: %.2f
              z: %.2f
              ]],
            head_position.x,
            head_position.y,
            head_position.z,
            virtual_head_position.x,
            virtual_head_position.y,
            virtual_head_position.z
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
    -- gizmo_draw(pass, mat4(gun_collider:getPose()), 1)

    pass:setColor(0.925, 0, 0.7)
    pass:draw(gun, mat4(automatic_gun_collider:getPose()))
    -- gizmo_draw(pass, mat4(gun_collider:getPose()), 1)


    local wallPos = vec3(wall:getPosition())
    pass:setColor(0.2, 0.2, 0.2)
    pass:box(wallPos.x, wallPos.y, wallPos.z, 10, 2, .2, wall:getOrientation())


    pass:setWireframe(wireframeEnabled)

    if phywireEnabled then
        phywire.draw(pass, world)
    end

    terrain_draw(pass)

    for i, bullet in ipairs(bullets) do
        pass:setColor(0, 1, 0)
        pass:sphere(vec3(bullet:getPosition()), 0.01, quat(bullet:getOrientation()))
    end

    for i, bottle in ipairs(bottles) do
        pass:setColor(0, 1, 1)
        pass:cylinder(vec3(bottle:getPosition()), 0.05, 0.5, quat(bottle:getOrientation()))
    end

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
