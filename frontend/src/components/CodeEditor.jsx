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
  Divider
} from "@chakra-ui/react";
import { Editor } from "@monaco-editor/react";
import RobotSelector from "./RobotSelector";
import { ROBOT_CODE_SNIPPETS } from "../constants";
import VideoPlayer from "./VideoPlayer";
import { motion } from "framer-motion";

// Motion components
const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);
const MotionHStack = motion(HStack);

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
        spacing={6}
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        transition={{ duration: 0.6 }}
      >
        {/* Session Header */}
        <MotionCard 
          w="full" 
          variant="glassmorphism"
          initial={{ opacity: 0, y: -20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, delay: 0.1 }}
        >
          <CardHeader>
            <HStack justify="space-between">
              <VStack align="start" spacing={3}>
                <HStack spacing={3}>
                  <Avatar size="md" name={user.name} />
                  <VStack align="start" spacing={0}>
                    <Text color="white" fontWeight="bold" fontSize="lg">{user.name}</Text>
                    <Text color="gray.400" fontSize="sm">{user.email}</Text>
                  </VStack>
                </HStack>
                
                {slot && (
                  <HStack spacing={4}>
                    <Badge 
                      variant="gradient" 
                      fontSize="sm"
                      px={3}
                      py={1}
                    >
                      Active Session
                    </Badge>
                    <HStack spacing={2}>
                      <MotionBox
                        fontSize="xl"
                        animate={{
                          rotate: [0, 10, -10, 0],
                          transition: {
                            duration: 3,
                            repeat: Infinity,
                          }
                        }}
                      >
                        {robotNames[slot.robotType].emoji}
                      </MotionBox>
                      <Text color="white" fontWeight="500">
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
              
              <HStack spacing={2}>
                <Button 
                  variant="glassmorphism" 
                  onClick={onBack} 
                  _hover={{
                    transform: "translateY(-2px)",
                    background: "rgba(255, 255, 255, 0.2)",
                  }}
                >
                  ← Back to Booking
                </Button>
                <Button 
                  variant="glassmorphism" 
                  onClick={onLogout}
                  _hover={{
                    transform: "translateY(-2px)",
                    background: "rgba(255, 255, 255, 0.2)",
                  }}
                >
                  Logout
                </Button>
              </HStack>
            </HStack>
          </CardHeader>
        </MotionCard>
        {/* Main Editor and Results */}
        <MotionCard 
          w="full" 
          variant="glassmorphism"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.6, delay: 0.2 }}
        >
          <CardHeader>
            <Text 
              fontSize="2xl" 
              fontWeight="bold" 
              bgGradient="linear(to-r, #667eea, #764ba2)"
              bgClip="text"
            >
              Robot Programming Console
            </Text>
          </CardHeader>
          <CardBody>
            <HStack spacing={8} align="start">
              {/* Code Editor Section */}
              <MotionBox 
                w="50%"
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ duration: 0.6, delay: 0.3 }}
              >
                <VStack spacing={4} align="start">
                  <RobotSelector robot={robot} onSelect={onSelect} />
                  <MotionBox 
                    w="full"
                    initial={{ opacity: 0, scale: 0.95 }}
                    animate={{ opacity: 1, scale: 1 }}
                    transition={{ duration: 0.6, delay: 0.4 }}
                  >
                    <Box
                      borderRadius="xl"
                      overflow="hidden"
                      border="1px solid rgba(255, 255, 255, 0.1)"
                      boxShadow="0 10px 25px rgba(0, 0, 0, 0.3)"
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
                          padding: { top: 16, bottom: 16 },
                          fontFamily: "'JetBrains Mono', 'Fira Code', 'Consolas', monospace",
                          cursorBlinking: "smooth",
                          smoothScrolling: true,
                        }}
                        height="75vh"
                        theme="vs-dark"
                        language="python"
                        defaultValue={ROBOT_CODE_SNIPPETS[robot]}
                        onMount={onMount}
                        value={value}
                        onChange={(value) => setValue(value)}
                      />
                    </Box>
                  </MotionBox>
                </VStack>
              </MotionBox>
              
              {/* Simulation Section */}
              <MotionBox 
                w="50%"
                initial={{ opacity: 0, x: 20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ duration: 0.6, delay: 0.5 }}
              >
                <VideoPlayer editorRef={editorRef} robot={robot} codeValue={value} />
              </MotionBox>
            </HStack>
          </CardBody>
        </MotionCard>
      </MotionVStack>
    </Container>
  );
};
export default CodeEditor;
