import { useRef, useState } from "react";
import { 
  Box, 
  HStack, 
  VStack, 
  Text, 
  Button, 
  Avatar, 
  Badge,
  Container,
  Card,
  CardBody,
  CardHeader,
  Divider,
  Flex,
  Spacer,
} from "@chakra-ui/react";
import { motion } from "framer-motion";
import { Editor } from "@monaco-editor/react";
import RobotSelector from "./RobotSelector";
import { ROBOT_CODE_SNIPPETS } from "../constants";
import VideoPlayer from "./VideoPlayer";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionText = motion(Text);
const MotionButton = motion(Button);
const MotionVStack = motion(VStack);

const robotNames = {
  turtlebot: { name: "TurtleBot3", emoji: "🤖" },
  arm: { name: "Robot Arm", emoji: "🦾" },
  hand: { name: "Robot Hand", emoji: "🤲" },
};

const CodeEditor = ({ user, slot, onBack, onLogout }) => {
  const editorRef = useRef();
  const [robot, setRobot] = useState(slot?.robotType || "turtlebot");
  const [value, setValue] = useState(ROBOT_CODE_SNIPPETS[slot?.robotType || "turtlebot"]);

  const onMount = (editor) => {
    editorRef.current = editor;
    editor.focus();
  };

  const onSelect = (robotType) => {
    setRobot(robotType);
    setValue(ROBOT_CODE_SNIPPETS[robotType]);
  };

  return (
    <Container maxW="7xl" py={8}>
      <MotionVStack 
        spacing={8}
        initial={{ opacity: 0, y: 50 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.8 }}
      >
        {/* Session Header */}
        <MotionCard 
          w="full" 
          variant="glass"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.2, duration: 0.8 }}
        >
          <CardHeader py={6}>
            <Flex justify="space-between" align="center">
              <VStack align="start" spacing={3}>
                <HStack spacing={3}>
                  <Avatar 
                    size="md" 
                    name={user.name}
                    border="2px solid"
                    borderColor="brand.400"
                  />
                  <VStack align="start" spacing={1}>
                    <Text color="white" fontWeight="bold" fontSize="lg">{user.name}</Text>
                    <Text color="gray.400" fontSize="sm">{user.email}</Text>
                  </VStack>
                </HStack>
                
                {slot && (
                  <HStack spacing={4} flexWrap="wrap">
                    <Badge variant="gradient" px={3} py={1} borderRadius="full">
                      🟢 Active Session
                    </Badge>
                    <HStack>
                      <Text fontSize="lg" filter="drop-shadow(0 0 5px rgba(56, 189, 248, 0.3))">
                        {robotNames[slot.robotType].emoji}
                      </Text>
                      <Text color="gray.300" fontWeight="600">
                        {robotNames[slot.robotType].name}
                      </Text>
                    </HStack>
                    <Text color="gray.400" fontSize="sm">
                      {new Date(slot.date).toLocaleDateString('en-US', { 
                        month: 'short', 
                        day: 'numeric' 
                      })} at {slot.startTime}
                    </Text>
                  </HStack>
                )}
              </VStack>
              
              <HStack spacing={3}>
                <MotionButton 
                  variant="glass" 
                  onClick={onBack} 
                  whileHover={{ scale: 1.05 }}
                  whileTap={{ scale: 0.95 }}
                >
                  ← Back to Booking
                </MotionButton>
                <MotionButton 
                  variant="glass" 
                  onClick={onLogout}
                  whileHover={{ scale: 1.05 }}
                  whileTap={{ scale: 0.95 }}
                >
                  Logout
                </MotionButton>
              </HStack>
            </Flex>
          </CardHeader>
        </MotionCard>

        {/* Main Editor and Results */}
        <MotionCard 
          w="full" 
          variant="glass"
          initial={{ opacity: 0, y: 30 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.4, duration: 0.8 }}
        >
          <CardHeader py={6}>
            <Flex align="center" gap={3}>
              <Box fontSize="xl">⚡</Box>
              <Text fontSize="xl" fontWeight="bold" color="white">
                Robot Programming Console
              </Text>
            </Flex>
          </CardHeader>
          <CardBody p={6}>
            <HStack spacing={8} align="start">
              {/* Code Editor Panel */}
              <MotionBox 
                w="50%"
                initial={{ opacity: 0, x: -30 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.6, duration: 0.8 }}
              >
                <VStack spacing={6} align="start">
                  <RobotSelector robot={robot} onSelect={onSelect} />
                  <MotionBox 
                    w="full" 
                    borderRadius="xl" 
                    overflow="hidden"
                    border="1px solid"
                    borderColor="rgba(255, 255, 255, 0.1)"
                    bg="rgba(0, 0, 0, 0.3)"
                    backdropFilter="blur(10px)"
                    whileHover={{ 
                      borderColor: "rgba(56, 189, 248, 0.3)",
                      boxShadow: "0 0 20px rgba(56, 189, 248, 0.1)"
                    }}
                    transition={{ duration: 0.3 }}
                  >
                    <Editor
                      options={{
                        minimap: {
                          enabled: false,
                        },
                        fontSize: 14,
                        lineNumbers: "on",
                        automaticLayout: true,
                        scrollBeyondLastLine: false,
                        theme: "vs-dark",
                        roundedSelection: false,
                        scrollbar: {
                          vertical: "visible",
                          horizontal: "visible",
                          useShadows: false,
                          verticalHasArrows: false,
                          horizontalHasArrows: false,
                        },
                      }}
                      height="75vh"
                      theme="vs-dark"
                      language="python"
                      defaultValue={ROBOT_CODE_SNIPPETS[robot]}
                      onMount={onMount}
                      value={value}
                      onChange={(value) => setValue(value)}
                    />
                  </MotionBox>
                </VStack>
              </MotionBox>
              
              {/* Video Results Panel */}
              <MotionBox 
                w="50%"
                initial={{ opacity: 0, x: 30 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.8, duration: 0.8 }}
              >
                <VideoPlayer 
                  editorRef={editorRef} 
                  robot={robot} 
                  codeValue={value} 
                />
              </MotionBox>
            </HStack>
          </CardBody>
        </MotionCard>
      </MotionVStack>
    </Container>
  );
};
export default CodeEditor;
