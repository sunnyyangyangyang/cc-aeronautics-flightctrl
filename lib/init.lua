-- Library loader for CC: Tweaked
-- Adds project directories to package.path

local root = "/home/cc/aeronautics-fc"  -- Adjust to your computer's path

-- Add lib and config directories to search path
package.path = package.path .. ";" .. root .. "/lib/?.lua"
package.path = package.path .. ";" .. root .. "/config/?.lua"

print("[Loader] Module paths configured for " .. root)
