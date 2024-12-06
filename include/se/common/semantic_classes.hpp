/*
 * SPDX-FileCopyrightText: 2022 Smart Robotics Lab, Imperial College London, Technical University of Munich
 * SPDX-FileCopyrightText: 2022 Nils Funk
 * SPDX-FileCopyrightText: 2022 Sotiris Papatheodorou
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef SE_SEMANTIC_CLASSES_HPP
#define SE_SEMANTIC_CLASSES_HPP

#include <array>

namespace se {
namespace classes {

inline constexpr std::array<const char*, 41> nyuv2 = {
    "background", "wall",    "floor",      "cabinet", "bed",    "chair",  "sofa",  "table",   "door",    "window", "bookshelf",      "picture",        "counter",  "blinds",
    "desk",       "shelves", "curtain",    "dresser", "pillow", "mirror", "floor", "clothes", "ceiling", "books",  "refridgerator",  "television",     "paper",    "towel",
    "shower",     "box",     "whiteboard", "person",  "night",  "toilet", "sink",  "lamp",    "bathtub", "bag",    "otherstructure", "otherfurniture", "otherprop"};

inline constexpr std::array<const char*, 42> mp3d = {
    "background", "wall", "floor",   "chair",    "door",   "table",         "picture", "cabinet",     "cushion",    "window",     "sofa",    "bed",     "curtain", "chest of drawers",
    "plant",      "sink", "stairs",  "ceiling",  "toilet", "stool",         "towel",   "mirror",      "tv monitor", "shower",     "column",  "bathtub", "counter", "fireplace",
    "lighting",   "beam", "railing", "shelving", "blinds", "gym equipment", "seating", "board panel", "furniture",  "appliances", "clothes", "objects", "misc",    "unlabel"};

inline constexpr std::array<const char*, 81> coco = {
    "background",    "person",     "bicycle",    "car",        "motorcycle", "airplane", "bus",       "train",        "truck",        "boat",         "traffic_light",  "fire_hydrant", "stop_sign",
    "parking_meter", "bench",      "bird",       "cat",        "dog",        "horse",    "sheep",     "cow",          "elephant",     "bear",         "zebra",          "giraffe",      "backpack",
    "umbrella",      "handbag",    "tie",        "suitcase",   "frisbee",    "skis",     "snowboard", "sports_ball",  "kite",         "baseball_bat", "baseball_glove", "skateboard",   "surfboard",
    "tennis_racket", "bottle",     "wine_glass", "cup",        "fork",       "knife",    "spoon",     "bowl",         "banana",       "apple",        "sandwich",       "orange",       "broccoli",
    "carrot",        "hot_dog",    "pizza",      "donut",      "cake",       "chair",    "couch",     "potted_plant", "bed",          "dining_table", "toilet",         "tv",           "laptop",
    "mouse",         "remote",     "keyboard",   "cell_phone", "microwave",  "oven",     "toaster",   "sink",         "refrigerator", "book",         "clock",          "vase",         "scissors",
    "teddy_bear",    "hair_drier", "toothbrush"};

inline constexpr std::array<const char*, 102> replica = {"background",
                                                         "backpack",
                                                         "base cabinet",
                                                         "basket",
                                                         "bathtub",
                                                         "beam",
                                                         "beanbag",
                                                         "bed",
                                                         "bench",
                                                         "bike",
                                                         "bin",
                                                         "blanket",
                                                         "blinds",
                                                         "book",
                                                         "bottle",
                                                         "box",
                                                         "bowl",
                                                         "camera",
                                                         "cabinet",
                                                         "candle",
                                                         "chair",
                                                         "chopping board",
                                                         "clock",
                                                         "cloth",
                                                         "clothing",
                                                         "coaster",
                                                         "comforter",
                                                         "computer keyboard",
                                                         "cup",
                                                         "cushion",
                                                         "curtain",
                                                         "ceiling",
                                                         "cooktop",
                                                         "countertop",
                                                         "desk",
                                                         "desk organizer",
                                                         "desktop computer",
                                                         "door",
                                                         "exercise ball",
                                                         "faucet",
                                                         "floor",
                                                         "handbag",
                                                         "hair dryer",
                                                         "handrail",
                                                         "indoor plant",
                                                         "knife block",
                                                         "kitchen utensil",
                                                         "lamp",
                                                         "laptop",
                                                         "major appliance",
                                                         "mat",
                                                         "microwave",
                                                         "monitor",
                                                         "mouse",
                                                         "nightstand",
                                                         "pan",
                                                         "panel",
                                                         "paper towel",
                                                         "phone",
                                                         "picture",
                                                         "pillar",
                                                         "pillow",
                                                         "pipe",
                                                         "plant stand",
                                                         "plate",
                                                         "pot",
                                                         "rack",
                                                         "refrigerator",
                                                         "remote control",
                                                         "scarf",
                                                         "sculpture",
                                                         "shelf",
                                                         "shoe",
                                                         "shower stall",
                                                         "sink",
                                                         "small appliance",
                                                         "sofa",
                                                         "stair",
                                                         "stool",
                                                         "switch",
                                                         "table",
                                                         "table runner",
                                                         "tablet",
                                                         "tissue paper",
                                                         "toilet",
                                                         "toothbrush",
                                                         "towel",
                                                         "tv screen",
                                                         "tv stand",
                                                         "umbrella",
                                                         "utensil holder",
                                                         "vase",
                                                         "vent",
                                                         "wall",
                                                         "wall cabinet",
                                                         "wall plug",
                                                         "wardrobe",
                                                         "window",
                                                         "rug",
                                                         "logo",
                                                         "bag",
                                                         "set of clothing"};

} // namespace classes
} // namespace se

#endif // SE_SEMANTIC_CLASSES_HPP
