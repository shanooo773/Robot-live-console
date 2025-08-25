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
      <VStack spacing={6}>
        {/* Session Header */}
        <Card w="full" bg="gray.800" border="1px solid" borderColor="gray.600">
          <CardHeader>
            <HStack justify="space-between">
              <VStack align="start" spacing={2}>
                <HStack>
                  <Avatar size="sm" name={user.name} />
                  <VStack align="start" spacing={0}>
                    <Text color="white" fontWeight="bold">{user.name}</Text>
                    <Text color="gray.400" fontSize="sm">{user.email}</Text>
                  </VStack>
                </HStack>
                
                {slot && (
                  <HStack spacing={4}>
                    <Badge colorScheme="green">Active Session</Badge>
                    <HStack>
                      <Text fontSize="lg">{robotNames[slot.robotType].emoji}</Text>
                      <Text color="gray.300">
                        {robotNames[slot.robotType].name}
                      </Text>
                    </HStack>
                    <Text color="gray.400">
                      {new Date(slot.date).toLocaleDateString('en-US', { 
                        month: 'short', 
                        day: 'numeric' 
                      })} at {slot.startTime}
                    </Text>
                  </HStack>
                )}
              </VStack>
              
              <HStack>
                <Button variant="ghost" onClick={onBack} color="gray.400">
                  ← Back to Booking
                </Button>
                <Button variant="ghost" onClick={onLogout} color="gray.400">
                  Logout
                </Button>
              </HStack>
            </HStack>
          </CardHeader>
        </Card>

        {/* Main Editor and Results */}
        <Card w="full" bg="gray.800" border="1px solid" borderColor="gray.600">
          <CardHeader>
            <Text fontSize="xl" fontWeight="bold" color="white">
              Robot Programming Console
            </Text>
          </CardHeader>
          <CardBody>
            <HStack spacing={6} align="start">
              <Box w="50%">
                <VStack spacing={4} align="start">
                  <RobotSelector robot={robot} onSelect={onSelect} />
                  <Box w="full">
                    <Editor
                      options={{
                        minimap: {
                          enabled: false,
                        },
                        fontSize: 14,
                        lineNumbers: "on",
                        automaticLayout: true,
                        scrollBeyondLastLine: false,
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
                </VStack>
              </Box>
              
              <Box w="50%">
                <VideoPlayer editorRef={editorRef} robot={robot} codeValue={value} />
              </Box>
            </HStack>
          </CardBody>
        </Card>
      </VStack>
    </Container>
  );
};
export default CodeEditor;
