const fs = require("fs")
const path = require("path")

const a = fs.readFileSync(path.resolve(__dirname, "main/Release/slaveControllerMain.bin"))
const b = fs.readFileSync(path.resolve(__dirname, "configurator/Release/slaveControllerConfigurator.bin"))
const result = Buffer.alloc(2048 + b.length)
a.copy(result)
b.copy(result, 2048)
fs.writeFileSync("firmware.bin", result)
