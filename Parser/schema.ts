import * as z from "zod";

const PacketSignalType = z.union([
    z.literal("uint"),
    z.literal("int"),
    z.literal("float"),
    z.literal("bool"),
]);
export type PacketSignalType = z.infer<typeof PacketSignalType>;

const PacketSignal = z.object({
    name: z.string(),
    start: z.int(),
    size: z.int(),
    type: PacketSignalType,
    min: z.number().optional(),
    max: z.number().optional(),
});
export type PacketSignal = z.infer<typeof PacketSignal>;

export const PacketDefs = z.object({
    version: z.string(),
    messages: z.array(z.object({
        name: z.string(),
        commandId: z.int(),
        signals: z.array(
            PacketSignal,
        ),
        comment: z.string().optional(),
    })),
});
export type PacketDefs = z.infer<typeof PacketDefs>;
