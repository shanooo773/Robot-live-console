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
  Icon,
  keyframes,
} from "@chakra-ui/react";
import { Editor } from "@monaco-editor/react";
import { motion } from "framer-motion";
import { FiCode, FiPlay, FiMonitor, FiUser, FiClock } from "react-icons/fi";
import RobotSelector from "./RobotSelector";
import { ROBOT_CODE_SNIPPETS } from "../constants";
import VideoPlayer from "./VideoPlayer";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);

// Animation keyframes
const glow = keyframes`
  0% { box-shadow: 0 0 20px rgba(67, 233, 123, 0.3); }
  50% { box-shadow: 0 0 30px rgba(67, 233, 123, 0.6); }
  100% { box-shadow: 0 0 20px rgba(67, 233, 123, 0.3); }
`;

const pulse = keyframes`
  0% { transform: scale(1); }
  50% { transform: scale(1.05); }
  100% { transform: scale(1); }
`;

const fadeInUp = {
  hidden: { opacity: 0, y: 20 },
  visible: { 
    opacity: 1, 
    y: 0,
    transition: { duration: 0.6, ease: "easeOut" }
  }
};

const staggerContainer = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.1,
      delayChildren: 0.2
    }
  }
};

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
        initial="hidden"
        animate="visible"
        variants={staggerContainer}
      >
        {/* Enhanced Session Header */}
        <MotionCard w="full" variant="floating" variants={fadeInUp}>
          <CardHeader>
            <HStack justify="space-between">
              <VStack align="start" spacing={3}>
                <HStack spacing={3}>
                  <Avatar 
                    size="md" 
                    name={user.name}
                    bg="linear-gradient(135deg, #667eea 0%, #764ba2 100%)"
                  />
                  <VStack align="start" spacing={0}>
                    <Text color="white" fontWeight="bold" fontSize="lg">{user.name}</Text>
                    <Text color="gray.400" fontSize="sm">{user.email}</Text>
                  </VStack>
                </HStack>
                
                {slot && (
                  <HStack spacing={6}>
                    <Badge 
                      variant="glass"
                      bg="rgba(67, 233, 123, 0.1)"
                      border="1px solid rgba(67, 233, 123, 0.3)"
                      color="green.300"
                      px={3}
                      py={1}
                      borderRadius="full"
                      animation={`${pulse} 2s infinite`}
                    >
                      <Icon as={FiMonitor} mr={1} />
                      Active Session
                    </Badge>
                    <HStack>
                      <Text fontSize="2xl">{robotNames[slot.robotType].emoji}</Text>
                      <Text color="white" fontWeight="medium">
                        {robotNames[slot.robotType].name}
                      </Text>
                    </HStack>
                    <HStack color="gray.400">
                      <Icon as={FiClock} />
                      <Text>
                        {new Date(slot.date).toLocaleDateString('en-US', { 
                          month: 'short', 
                          day: 'numeric' 
                        })} at {slot.startTime}
                      </Text>
                    </HStack>
                  </HStack>
                )}
              </VStack>
              
              <HStack spacing={3}>
                <Button 
                  variant="glass" 
                  onClick={onBack}
                  size="lg"
                >
                  ← Back to Booking
                </Button>
                <Button 
                  variant="glass" 
                  onClick={onLogout}
                  size="lg"
                >
                  Logout
                </Button>
              </HStack>
            </HStack>
          </CardHeader>
        </MotionCard>

        {/* Enhanced Main Editor and Results */}
        <MotionCard w="full" variant="floating" variants={fadeInUp}>
          <CardHeader>
            <HStack>
              <Icon as={FiCode} color="purple.300" boxSize={6} />
              <Text 
                fontSize="xl" 
                fontWeight="bold" 
                bgGradient="linear(to-r, #667eea, #764ba2)"
                bgClip="text"
              >
                Robot Programming Console
              </Text>
            </HStack>
          </CardHeader>
          <CardBody>
            <HStack spacing={8} align="start">
              <MotionBox 
                w="50%"
                initial={{ opacity: 0, x: -20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.3, duration: 0.6 }}
              >
                <VStack spacing={6} align="start">
                  <Box w="full">
                    <RobotSelector robot={robot} onSelect={onSelect} />
                  </Box>
                  <MotionBox 
                    w="full"
                    borderRadius="xl"
                    overflow="hidden"
                    border="1px solid rgba(255, 255, 255, 0.1)"
                    boxShadow="0 10px 30px rgba(0, 0, 0, 0.3)"
                    whileHover={{ 
                      boxShadow: "0 15px 40px rgba(102, 126, 234, 0.2)",
                      transition: { duration: 0.3 }
                    }}
                  >
                    <Editor
                      options={{
                        minimap: {
                          enabled: false,
                        },
                        fontSize: 16,
                        lineNumbers: "on",
                        automaticLayout: true,
                        scrollBeyondLastLine: false,
                        fontFamily: "'JetBrains Mono', 'Monaco', 'Menlo', monospace",
                        theme: "vs-dark",
                        cursorBlinking: "smooth",
                        renderLineHighlight: "gutter",
                        selectOnLineNumbers: true,
                        glyphMargin: true,
                        folding: true,
                        foldingStrategy: "indentation",
                        showFoldingControls: "always",
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
              
              <MotionBox 
                w="50%"
                initial={{ opacity: 0, x: 20 }}
                animate={{ opacity: 1, x: 0 }}
                transition={{ delay: 0.5, duration: 0.6 }}
              >
                <VStack spacing={4} align="start">
                  <HStack>
                    <Icon as={FiMonitor} color="green.300" boxSize={5} />
                    <Text 
                      fontSize="lg" 
                      fontWeight="bold" 
                      bgGradient="linear(to-r, #43e97b, #38f9d7)"
                      bgClip="text"
                    >
                      Robot Simulation
                    </Text>
                  </HStack>
                  <VideoPlayer 
                    editorRef={editorRef}
                    robot={robot}
                    codeValue={value}
                  />
                </VStack>
              </MotionBox>
            </HStack>
          </CardBody>
        </MotionCard>
      </MotionVStack>
    </Container>
  );
};

export default CodeEditor;
