local NODES = {
    [0x03] = "Front_Steering",
    [0x07] = "Rear_Steering",
    [0x04] = "Front_Traction",
    [0x08] = "Rear_Traction"
}

local PARAM_MAP = {
    [0x6040] = { sub = 0x00, name = "Controlword" },
    [0x6060] = { sub = 0x00, name = "Traction_Mode" },
    [0x3003] = { sub = 0x00, name = "Steering_Mode" },
    [0x3004] = { sub = 0x00, name = "Steering_State" },
    [0x60FF] = { sub = 0x00, name = "Traction_Velocity" },
    [0x3300] = { sub = 0x00, name = "Steering_Profile_Velocity" },
    [0x3790] = { sub = 0x00, name = "Steering_Position" },

    -- Feedback Indices (from 0x43 Replies)
    [0x6041] = { sub = 0x00, name = "Traction_Statusword" },
    [0x606C] = { sub = 0x01, name = "Traction_Velocity" },
    [0x3A04] = { sub = 0x01, name = "Steering_Velocity" },
    [0x3762] = { sub = 0x00, name = "Steering_Position" }
}

-- SDO Command Specifiers
local CMD_WRITE_4B = 0x23 -- Setpoint (Master to Slave)
local CMD_REPLY_4B = 0x43 -- Feedback (Slave to Master)

-- Helper function to decode a specific CAN stream
local function ProcessCANStream(topic_prefix, is_rx, output_series)
    local id_series = TimeseriesView.find(topic_prefix .. "/id")
    local data = {}

    for b = 0, 7 do
        data[b] = TimeseriesView.find(topic_prefix .. "/data." .. b)
    end

    if id_series == nil then return end

    for i = 0, id_series:size() - 1 do
        local timestamp = id_series:atTime(i)
        local can_id = id_series:at(i)

        -- =========================================================
        -- CAN ID DECODING: 0x580 for Feedback, 0x600 for Setpoints
        -- =========================================================
        local base_id = is_rx and 0x580 or 0x600
        local node_id = can_id - base_id

        local motor_name = NODES[node_id]

        -- If the calculated Node ID matches our dictionary, process it
        if motor_name ~= nil then

            -- Safely extract Bytes 0 to 3
            local byte0 = (data[0] ~= nil) and data[0]:atTime(timestamp) or 0
            local byte1 = (data[1] ~= nil) and data[1]:atTime(timestamp) or 0
            local byte2 = (data[2] ~= nil) and data[2]:atTime(timestamp) or 0
            local byte3 = (data[3] ~= nil) and data[3]:atTime(timestamp) or 0

            if output_series['byte0'] == nil then
              output_series['byte0'] = Timeseries.new(target_series_name)
            end

            output_series['byte0']:push_back(byte0)
            -- Calculate Parameter Index (Little Endian: Byte 1 + Byte 2 shifted)
            local index = byte1 + (byte2 * 256)
            local subindex = byte3

            local is_valid_msg = false
            local msg_type = ""

            -- Classify as Setpoint or Feedback
            if not is_rx and byte0 == CMD_WRITE_4B then
                is_valid_msg = true
                msg_type = "_Setpoint"
            elseif is_rx and byte0 == CMD_REPLY_4B then
                is_valid_msg = true
                msg_type = "_Feedback"
            end

            -- Match the Index/Subindex to our dictionary
            local param_info = PARAM_MAP[index]

            if is_valid_msg and param_info ~= nil and param_info.sub == subindex then

                -- Construct the target plot name (e.g., "Steering_1_Velocity_Feedback")
                local target_series_name = motor_name .. "_" .. param_info.name .. msg_type

                -- Ensure series exists in our output table
                if output_series[target_series_name] == nil then
                    output_series[target_series_name] = Timeseries.new(target_series_name)
                end

                -- Extract 32-bit Data Payload (Bytes 4, 5, 6, 7)
                local b4 = (data[4] ~= nil) and data[4]:atTime(timestamp) or 0
                local b5 = (data[5] ~= nil) and data[5]:atTime(timestamp) or 0
                local b6 = (data[6] ~= nil) and data[6]:atTime(timestamp) or 0
                local b7 = (data[7] ~= nil) and data[7]:atTime(timestamp) or 0

                local extracted_value = b4 + (b5 * 256) + (b6 * 65536) + (b7 * 16777216)

                -- Convert to signed 32-bit integer
                if extracted_value >= 2147483648 then
                    extracted_value = extracted_value - 4294967296
                end

                -- Push the value to the plot
                output_series[target_series_name]:push_back(timestamp, 1.0)
            end
        end
    end
end

-- Main Function
function DecodeAllMotors(can_rx_topic, can_tx_topic)
    local output_series = {}

    -- Process the Receive (Feedback) stream [is_rx = true]
    -- This flags the processor to look for 0x580 + Node IDs
    ProcessCANStream(can_rx_topic, true, output_series)

    -- Process the Transmit (Setpoint) stream [is_rx = false]
    -- This flags the processor to look for 0x600 + Node IDs
    ProcessCANStream(can_tx_topic, false, output_series)

    return output_series
end

-- ==============================================================================
-- EXECUTE SCRIPT
-- ==============================================================================
return DecodeAllMotors("mobotic/can_rx", "mobotic/can_tx")
