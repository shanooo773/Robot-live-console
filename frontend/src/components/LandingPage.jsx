import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Container,
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalBody,
  ModalCloseButton,
  useDisclosure,
  SimpleGrid,
  Card,
  CardBody,
  Image,
  Badge,
} from "@chakra-ui/react";
import { useState } from "react";

const robotTypes = [
  {
    id: "turtlebot",
    name: "TurtleBot3",
    description: "Mobile robot for navigation and path planning",
    image: "🤖",
    features: ["Autonomous Navigation", "SLAM Mapping", "Obstacle Avoidance"]
  },
  {
    id: "arm",
    name: "Robot Arm",
    description: "6-DOF manipulator for pick and place operations",
    image: "🦾",
    features: ["6 Degrees of Freedom", "Precise Positioning", "Object Manipulation"]
  },
  {
    id: "hand",
    name: "Robot Hand",
    description: "Dexterous gripper for complex manipulation tasks",
    image: "🤲",
    features: ["Multi-finger Grasping", "Force Feedback", "Adaptive Grip"]
  }
];

const LandingPage = ({ onGetStarted }) => {
  const { isOpen, onOpen, onClose } = useDisclosure();
  const [selectedRobot, setSelectedRobot] = useState(null);

  const handleRobotClick = (robot) => {
    setSelectedRobot(robot);
    onOpen();
  };

  return (
    <Box minH="100vh">
      {/* Animated Background */}
      <Box
        position="absolute"
        top={0}
        left={0}
        right={0}
        bottom={0}
        bgGradient="gradient.dark"
        opacity={0.9}
        zIndex={-1}
      />
      
      {/* Floating particles effect */}
      <Box
        position="absolute"
        top={0}
        left={0}
        right={0}
        bottom={0}
        overflow="hidden"
        zIndex={-1}
      >
        {[...Array(20)].map((_, i) => (
          <Box
            key={i}
            position="absolute"
            w="4px"
            h="4px"
            bg="robotics.primary"
            borderRadius="full"
            opacity={0.3}
            animation={`float ${3 + i * 0.5}s ease-in-out infinite alternate`}
            style={{
              left: `${Math.random() * 100}%`,
              top: `${Math.random() * 100}%`,
              animationDelay: `${i * 0.2}s`,
            }}
          />
        ))}
      </Box>

      <Container maxW="7xl" py={20} position="relative" zIndex={1}>
        <VStack spacing={20} align="center">
          {/* Enhanced Hero Section */}
          <VStack spacing={8} textAlign="center">
            <Box position="relative">
              <Text fontSize="8xl" mb={4}>🤖</Text>
              <Box
                position="absolute"
                top="50%"
                left="50%"
                transform="translate(-50%, -50%)"
                w="200px"
                h="200px"
                bgGradient="radial(circle, robotics.primary 0%, transparent 70%)"
                opacity={0.3}
                borderRadius="full"
                animation="pulse 2s ease-in-out infinite"
              />
            </Box>
            
            <VStack spacing={4}>
              <Text 
                fontSize={{ base: "3xl", md: "5xl", lg: "6xl" }} 
                fontWeight="bold" 
                bgGradient="linear(to-r, white, robotics.primary)"
                bgClip="text"
                lineHeight="shorter"
              >
                Robot Programming Console
              </Text>
              <Text 
                fontSize={{ base: "md", md: "lg", lg: "xl" }} 
                color="gray.300" 
                maxW="3xl"
                lineHeight="tall"
              >
                Experience the future of robotics education. Book your time slot to program 
                real robots in simulation, write Python code, control robotic systems, 
                and see live results in an immersive environment.
              </Text>
            </VStack>
            
            <VStack spacing={4}>
              <Button
                variant="gradient"
                size="lg"
                onClick={onGetStarted}
                fontSize="lg"
                px={12}
                py={8}
                h="auto"
                boxShadow="glow"
                _hover={{
                  boxShadow: "glow-lg",
                }}
              >
                Start Your Robotics Journey
              </Button>
              
              <HStack spacing={8} color="gray.400" fontSize="sm">
                <HStack>
                  <Text>⚡</Text>
                  <Text>Real-time Simulation</Text>
                </HStack>
                <HStack>
                  <Text>🐍</Text>
                  <Text>Python Programming</Text>
                </HStack>
                <HStack>
                  <Text>🎥</Text>
                  <Text>Live Video Feed</Text>
                </HStack>
              </HStack>
            </VStack>
          </VStack>

        {/* Enhanced How It Works Section */}
        <VStack spacing={12} w="full">
          <VStack spacing={4} textAlign="center">
            <Text 
              fontSize={{ base: "3xl", md: "4xl" }} 
              fontWeight="bold" 
              bgGradient="linear(to-r, white, robotics.accent)"
              bgClip="text"
            >
              How It Works
            </Text>
            <Text fontSize="lg" color="gray.400" maxW="2xl">
              Get started with robotics programming in three simple steps
            </Text>
          </VStack>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            <Card variant="gradient" position="relative">
              <CardBody textAlign="center" p={8}>
                <Box position="relative" mb={6}>
                  <Text fontSize="5xl" mb={2}>📅</Text>
                  <Badge 
                    variant="robotics" 
                    position="absolute" 
                    top={0} 
                    right={0}
                    borderRadius="full"
                    px={3}
                  >
                    Step 1
                  </Badge>
                </Box>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={3}>
                  Book a Time Slot
                </Text>
                <Text color="gray.300" lineHeight="tall">
                  Choose an available time slot that fits your schedule. 
                  Select from various robot types and simulation environments.
                </Text>
              </CardBody>
            </Card>

            <Card variant="gradient" position="relative">
              <CardBody textAlign="center" p={8}>
                <Box position="relative" mb={6}>
                  <Text fontSize="5xl" mb={2}>💻</Text>
                  <Badge 
                    variant="robotics" 
                    position="absolute" 
                    top={0} 
                    right={0}
                    borderRadius="full"
                    px={3}
                  >
                    Step 2
                  </Badge>
                </Box>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={3}>
                  Write Python Code
                </Text>
                <Text color="gray.300" lineHeight="tall">
                  Use our Monaco editor to write ROS Python code for robots. 
                  Access comprehensive libraries and real-time syntax checking.
                </Text>
              </CardBody>
            </Card>

            <Card variant="gradient" position="relative">
              <CardBody textAlign="center" p={8}>
                <Box position="relative" mb={6}>
                  <Text fontSize="5xl" mb={2}>🎥</Text>
                  <Badge 
                    variant="robotics" 
                    position="absolute" 
                    top={0} 
                    right={0}
                    borderRadius="full"
                    px={3}
                  >
                    Step 3
                  </Badge>
                </Box>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={3}>
                  See Results
                </Text>
                <Text color="gray.300" lineHeight="tall">
                  Watch your robot perform tasks in real-time simulation. 
                  Get instant feedback and video recordings of your sessions.
                </Text>
              </CardBody>
            </Card>
          </SimpleGrid>
        </VStack>

        {/* Enhanced Available Robots Section */}
        <VStack spacing={12} w="full">
          <VStack spacing={4} textAlign="center">
            <Text 
              fontSize={{ base: "3xl", md: "4xl" }} 
              fontWeight="bold" 
              bgGradient="linear(to-r, white, robotics.secondary)"
              bgClip="text"
            >
              Available Robots
            </Text>
            <Text fontSize="lg" color="gray.400" maxW="2xl">
              Choose from our diverse fleet of robotic platforms, each designed for specific tasks
            </Text>
          </VStack>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            {robotTypes.map((robot, index) => (
              <Card 
                key={robot.id} 
                variant="gradient"
                cursor="pointer"
                onClick={() => handleRobotClick(robot)}
                position="relative"
                overflow="hidden"
                _hover={{ 
                  transform: "translateY(-8px) scale(1.02)",
                  boxShadow: "glow",
                }}
                transition="all 0.3s ease-in-out"
                style={{
                  animationDelay: `${index * 0.1}s`,
                }}
              >
                {/* Hover effect overlay */}
                <Box
                  position="absolute"
                  top={0}
                  left={0}
                  right={0}
                  bottom={0}
                  bgGradient="linear(to-br, rgba(0,212,255,0.1), rgba(255,107,107,0.1))"
                  opacity={0}
                  transition="opacity 0.3s"
                  _groupHover={{ opacity: 1 }}
                />
                
                <CardBody textAlign="center" p={8} position="relative" zIndex={1}>
                  <Box position="relative" mb={6}>
                    <Text fontSize="7xl" mb={2} display="inline-block" transition="transform 0.3s">
                      {robot.image}
                    </Text>
                    <Box
                      position="absolute"
                      top="50%"
                      left="50%"
                      transform="translate(-50%, -50%)"
                      w="120px"
                      h="120px"
                      bgGradient="radial(circle, robotics.accent 0%, transparent 70%)"
                      opacity={0}
                      borderRadius="full"
                      transition="opacity 0.3s"
                      _groupHover={{ opacity: 0.3 }}
                    />
                  </Box>
                  
                  <VStack spacing={3} mb={6}>
                    <Text fontSize="2xl" fontWeight="bold" color="white">
                      {robot.name}
                    </Text>
                    <Text color="gray.300" lineHeight="tall" fontSize="md">
                      {robot.description}
                    </Text>
                  </VStack>
                  
                  <VStack spacing={3}>
                    <Text fontSize="sm" fontWeight="semibold" color="robotics.primary" mb={2}>
                      KEY FEATURES
                    </Text>
                    <SimpleGrid columns={1} spacing={2} w="full">
                      {robot.features.map((feature, featureIndex) => (
                        <Badge 
                          key={featureIndex} 
                          variant="glow" 
                          fontSize="xs"
                          py={1}
                          px={3}
                          borderRadius="md"
                        >
                          {feature}
                        </Badge>
                      ))}
                    </SimpleGrid>
                  </VStack>
                </CardBody>
              </Card>
            ))}
          </SimpleGrid>
        </VStack>

        {/* Enhanced Call to Action */}
        <Box 
          w="full" 
          textAlign="center" 
          py={16}
          px={8}
          bgGradient="gradient.card"
          borderRadius="2xl"
          border="1px solid"
          borderColor="whiteAlpha.200"
          position="relative"
          overflow="hidden"
        >
          {/* Background animation */}
          <Box
            position="absolute"
            top={0}
            left={0}
            right={0}
            bottom={0}
            bgGradient="linear(45deg, transparent 30%, rgba(0,212,255,0.1) 50%, transparent 70%)"
            animation="slideBackground 3s ease-in-out infinite alternate"
          />
          
          <VStack spacing={8} position="relative" zIndex={1}>
            <VStack spacing={4}>
              <Text 
                fontSize={{ base: "2xl", md: "3xl", lg: "4xl" }} 
                fontWeight="bold" 
                bgGradient="linear(to-r, white, robotics.primary, robotics.accent)"
                bgClip="text"
                lineHeight="shorter"
              >
                Ready to Start Programming Robots?
              </Text>
              <Text fontSize="lg" color="gray.300" maxW="2xl">
                Join thousands of students and professionals learning robotics through hands-on programming. 
                Your journey into the future of automation starts here.
              </Text>
            </VStack>
            
            <HStack spacing={4} flexWrap="wrap" justify="center">
              <Button
                variant="robotics"
                size="lg"
                onClick={onGetStarted}
                fontSize="lg"
                px={12}
                py={8}
                h="auto"
                leftIcon={<Text fontSize="xl">🚀</Text>}
              >
                Book Your Session Now
              </Button>
              
              <Button
                variant="outline"
                size="lg"
                fontSize="lg"
                px={8}
                py={8}
                h="auto"
                color="robotics.primary"
                borderColor="robotics.primary"
                _hover={{
                  bg: "rgba(0,212,255,0.1)",
                  transform: "translateY(-2px)",
                }}
                leftIcon={<Text fontSize="xl">📖</Text>}
              >
                Learn More
              </Button>
            </HStack>
            
            <HStack spacing={8} color="gray.400" fontSize="sm" flexWrap="wrap" justify="center">
              <HStack>
                <Text fontSize="lg">✨</Text>
                <Text>No Setup Required</Text>
              </HStack>
              <HStack>
                <Text fontSize="lg">🎓</Text>
                <Text>Expert Support</Text>
              </HStack>
              <HStack>
                <Text fontSize="lg">⏰</Text>
                <Text>24/7 Access</Text>
              </HStack>
            </HStack>
          </VStack>
        </Box>
      </VStack>
      </Container>

      {/* Enhanced Robot Details Modal */}
      <Modal isOpen={isOpen} onClose={onClose} size="xl" isCentered>
        <ModalOverlay bg="blackAlpha.700" backdropFilter="blur(4px)" />
        <ModalContent 
          bg="dark.surface" 
          color="white"
          border="1px solid"
          borderColor="dark.border"
          borderRadius="xl"
          boxShadow="0 20px 60px rgba(0, 0, 0, 0.5)"
        >
          <ModalHeader p={8}>
            <HStack spacing={4}>
              <Box position="relative">
                <Text fontSize="4xl">{selectedRobot?.image}</Text>
                <Box
                  position="absolute"
                  top="50%"
                  left="50%"
                  transform="translate(-50%, -50%)"
                  w="80px"
                  h="80px"
                  bgGradient="radial(circle, robotics.primary 0%, transparent 70%)"
                  opacity={0.3}
                  borderRadius="full"
                  animation="pulse 2s ease-in-out infinite"
                />
              </Box>
              <VStack align="start" spacing={1}>
                <Text fontSize="2xl" fontWeight="bold">{selectedRobot?.name}</Text>
                <Badge variant="robotics" fontSize="xs">
                  AVAILABLE NOW
                </Badge>
              </VStack>
            </HStack>
          </ModalHeader>
          <ModalCloseButton 
            size="lg" 
            color="gray.400"
            _hover={{ color: "robotics.primary", bg: "whiteAlpha.100" }}
          />
          <ModalBody pb={8} px={8}>
            <VStack spacing={6} align="start">
              <Text color="gray.300" fontSize="lg" lineHeight="tall">
                {selectedRobot?.description}
              </Text>
              
              <Box w="full">
                <Text fontWeight="bold" mb={4} color="robotics.primary" fontSize="lg">
                  Key Features & Capabilities
                </Text>
                <SimpleGrid columns={1} spacing={3}>
                  {selectedRobot?.features.map((feature, index) => (
                    <HStack key={index} p={3} bg="whiteAlpha.50" borderRadius="md">
                      <Box
                        w="6px"
                        h="6px"
                        bg="robotics.primary"
                        borderRadius="full"
                        boxShadow="0 0 10px rgba(0,212,255,0.5)"
                      />
                      <Text fontSize="md">{feature}</Text>
                    </HStack>
                  ))}
                </SimpleGrid>
              </Box>

              <Box w="full" pt={4}>
                <Button
                  variant="robotics"
                  onClick={() => {
                    onClose();
                    onGetStarted();
                  }}
                  w="full"
                  size="lg"
                  py={6}
                  h="auto"
                  leftIcon={<Text fontSize="xl">🚀</Text>}
                >
                  Start Programming This Robot
                </Button>
              </Box>
            </VStack>
          </ModalBody>
        </ModalContent>
      </Modal>
    </Box>
  );
};

export default LandingPage;