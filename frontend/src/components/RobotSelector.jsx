import { Box, Button, Menu, MenuButton, MenuList, MenuItem, Text, HStack } from "@chakra-ui/react";
import { motion } from "framer-motion";
import { useState } from "react";

const MotionBox = motion(Box);
const MotionButton = motion(MenuButton);

const ROBOT_NAMES = {
  arm: { name: "Robot Arm", emoji: "🦾" },
  hand: { name: "Robot Hand", emoji: "🤲" }, 
  turtlebot: { name: "TurtleBot3", emoji: "🤖" }
};

const RobotSelector = ({ robot, onSelect }) => {
  return (
    <MotionBox 
      mb={4} 
      minW="200px"
      initial={{ opacity: 0, y: 10 }}
      animate={{ opacity: 1, y: 0 }}
      transition={{ duration: 0.5 }}
    >
      <Text mb={3} fontSize="md" color="gray.300" fontWeight="600">
        🔧 Robot Type:
      </Text>
      <Menu isLazy>
        <MotionButton 
          as={Button} 
          variant="glass"
          size="lg"
          w="full"
          justifyContent="flex-start"
          whileHover={{ scale: 1.02 }}
          whileTap={{ scale: 0.98 }}
        >
          <HStack>
            <Text fontSize="lg">{ROBOT_NAMES[robot]?.emoji}</Text>
            <Text>{ROBOT_NAMES[robot]?.name || "Select Robot"}</Text>
          </HStack>
        </MotionButton>
        <MenuList 
          bg="rgba(15, 23, 42, 0.95)" 
          border="1px solid rgba(255, 255, 255, 0.1)"
          borderRadius="xl"
          backdropFilter="blur(20px)"
          p={2}
        >
          {Object.entries(ROBOT_NAMES).map(([robotType, robotData]) => (
            <MenuItem
              key={robotType}
              color={robotType === robot ? "white" : "gray.300"}
              bg={robotType === robot ? "rgba(56, 189, 248, 0.2)" : "transparent"}
              borderRadius="lg"
              _hover={{
                color: "white",
                bg: "rgba(56, 189, 248, 0.1)",
              }}
              onClick={() => onSelect(robotType)}
              py={3}
              px={4}
            >
              <HStack>
                <Text fontSize="lg">{robotData.emoji}</Text>
                <Text>{robotData.name}</Text>
              </HStack>
            </MenuItem>
          ))}
        </MenuList>
      </Menu>
    </MotionBox>
  );
};

export default RobotSelector;