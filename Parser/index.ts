import * as fs from "fs";
import * as yaml from "yaml";
import { PacketDefs } from "./schema";
import { genC } from "./gen-c";

// path to packet definition YAML file
const configPath = "./packets.yaml";

// read packet definitions from disk
const configRaw = fs.readFileSync(configPath, "utf-8");

// parse packet definitions for valid syntax
const configObject = yaml.parse(configRaw);
const packetConfig = PacketDefs.parse(configObject);

const args = process.argv.slice(2);
switch (args[0]) {
    // parse only to check for valid syntax, which we achieved if we ran to here
    // TODO: enforce rules such as bit overlap
    case "lint":
        console.log(`${configPath}:\n\tSyntax is valid and all required fields are present`)
        process.exit(0);  // exit with success

    // generate C headers
    case "gen":
        genC(packetConfig);
        break;

    default:
        console.error(`Unsupported command line argument ${args[0]}`);
        process.exit(1);  // exit with error
}
