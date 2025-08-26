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
  Icon,
} from "@chakra-ui/react";
import { motion, useAnimation } from "framer-motion";
import { useState, useEffect } from "react";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionText = motion(Text);
const MotionButton = motion(Button);

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
  const controls = useAnimation();

  const handleRobotClick = (robot) => {
    setSelectedRobot(robot);
    onOpen();
  };

  useEffect(() => {
    controls.start({
      y: [0, -10, 0],
      transition: {
        duration: 2,
        repeat: Infinity,
        ease: "easeInOut"
      }
    });
  }, [controls]);

  return (
    <Container maxW="7xl" py={20}>
      <VStack spacing={20} align="center">
        {/* Hero Section */}
        <MotionBox
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
        >
          <VStack spacing={8} textAlign="center">
            <MotionText 
              fontSize="8xl" 
              animate={controls}
              filter="drop-shadow(0 0 20px rgba(56, 189, 248, 0.3))"
            >
              🤖
            </MotionText>
            <MotionText 
              fontSize={{ base: "3xl", md: "5xl", lg: "6xl" }} 
              fontWeight="bold" 
              bgGradient="linear(to-r, brand.400, accent.400)"
              bgClip="text"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.3, duration: 0.8 }}
            >
              Robot Programming Console
            </MotionText>
            <MotionText 
              fontSize={{ base: "lg", md: "xl" }}
              color="gray.300" 
              maxW="3xl"
              lineHeight="1.6"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              transition={{ delay: 0.5, duration: 0.8 }}
            >
              Experience the future of robotics education. Book your time slot to program real robots in simulation. 
              Write Python code, control advanced robots, and see live results in real-time.
            </MotionText>
            <MotionButton
              size="lg"
              variant="gradient"
              onClick={onGetStarted}
              fontSize="lg"
              px={12}
              py={8}
              h="auto"
              initial={{ opacity: 0, scale: 0.8 }}
              animate={{ opacity: 1, scale: 1 }}
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
              transition={{ delay: 0.7, duration: 0.8 }}
            >
              🚀 Start Your Journey
            </MotionButton>
          </VStack>
        </MotionBox>

        {/* How It Works */}
        <MotionBox
          w="full"
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.9, duration: 0.8 }}
        >
          <VStack spacing={12} w="full">
            <MotionText 
              fontSize={{ base: "2xl", md: "4xl" }} 
              fontWeight="bold" 
              color="white"
              textAlign="center"
            >
              How It Works
            </MotionText>
            
            <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
              <MotionCard 
                variant="glass"
                whileHover={{ y: -8, scale: 1.02 }}
                transition={{ type: "spring", stiffness: 300 }}
                initial={{ opacity: 0, y: 30 }}
                animate={{ opacity: 1, y: 0 }}
                custom={0}
              >
                <CardBody textAlign="center" py={8}>
                  <Box
                    fontSize="5xl"
                    mb={6}
                    filter="drop-shadow(0 0 10px rgba(34, 197, 94, 0.3))"
                  >
                    📅
                  </Box>
                  <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                    1. Book Your Session
                  </Text>
                  <Text color="gray.300" lineHeight="1.6">
                    Choose your preferred time slot and robot type for an immersive programming experience
                  </Text>
                </CardBody>
              </MotionCard>

              <MotionCard 
                variant="glass"
                whileHover={{ y: -8, scale: 1.02 }}
                transition={{ type: "spring", stiffness: 300 }}
                initial={{ opacity: 0, y: 30 }}
                animate={{ opacity: 1, y: 0 }}
                custom={1}
              >
                <CardBody textAlign="center" py={8}>
                  <Box
                    fontSize="5xl"
                    mb={6}
                    filter="drop-shadow(0 0 10px rgba(56, 189, 248, 0.3))"
                  >
                    💻
                  </Box>
                  <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                    2. Write Python Code
                  </Text>
                  <Text color="gray.300" lineHeight="1.6">
                    Use our advanced Monaco editor to write ROS Python code with syntax highlighting and autocomplete
                  </Text>
                </CardBody>
              </MotionCard>

              <MotionCard 
                variant="glass"
                whileHover={{ y: -8, scale: 1.02 }}
                transition={{ type: "spring", stiffness: 300 }}
                initial={{ opacity: 0, y: 30 }}
                animate={{ opacity: 1, y: 0 }}
                custom={2}
              >
                <CardBody textAlign="center" py={8}>
                  <Box
                    fontSize="5xl"
                    mb={6}
                    filter="drop-shadow(0 0 10px rgba(217, 70, 239, 0.3))"
                  >
                    🎥
                  </Box>
                  <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                    3. See Results Live
                  </Text>
                  <Text color="gray.300" lineHeight="1.6">
                    Watch your robot perform tasks in real-time simulation with instant visual feedback
                  </Text>
                </CardBody>
              </MotionCard>
            </SimpleGrid>
          </VStack>
        </MotionBox>

        {/* Available Robots */}
        <MotionBox
          w="full"
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 1.1, duration: 0.8 }}
        >
          <VStack spacing={12} w="full">
            <MotionText 
              fontSize={{ base: "2xl", md: "4xl" }} 
              fontWeight="bold" 
              color="white"
              textAlign="center"
            >
              Available Robots
            </MotionText>
            
            <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
              {robotTypes.map((robot, index) => (
                <MotionCard 
                  key={robot.id} 
                  variant="glass"
                  cursor="pointer"
                  onClick={() => handleRobotClick(robot)}
                  whileHover={{ 
                    y: -8, 
                    scale: 1.02,
                    rotate: [0, 1, -1, 0],
                  }}
                  transition={{ 
                    type: "spring", 
                    stiffness: 300,
                    rotate: { duration: 0.6 }
                  }}
                  initial={{ opacity: 0, y: 30 }}
                  animate={{ opacity: 1, y: 0 }}
                  custom={index}
                >
                  <CardBody textAlign="center" py={8}>
                    <MotionBox
                      fontSize="6xl"
                      mb={6}
                      whileHover={{ scale: 1.2, rotate: 360 }}
                      transition={{ duration: 0.6 }}
                      filter="drop-shadow(0 0 15px rgba(56, 189, 248, 0.2))"
                    >
                      {robot.image}
                    </MotionBox>
                    <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                      {robot.name}
                    </Text>
                    <Text color="gray.300" mb={6} lineHeight="1.6">
                      {robot.description}
                    </Text>
                    <VStack spacing={3}>
                      {robot.features.map((feature, index) => (
                        <Badge 
                          key={index} 
                          variant="glass" 
                          px={4}
                          py={2}
                          fontSize="sm"
                        >
                          {feature}
                        </Badge>
                      ))}
                    </VStack>
                  </CardBody>
                </MotionCard>
              ))}
            </SimpleGrid>
          </VStack>
        </MotionBox>

        {/* Call to Action */}
        <MotionBox
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 1.3, duration: 0.8 }}
        >
          <VStack spacing={8} textAlign="center" py={12}>
            <MotionText 
              fontSize={{ base: "xl", md: "3xl" }} 
              fontWeight="bold" 
              bgGradient="linear(to-r, brand.400, accent.400)"
              bgClip="text"
            >
              Ready to start programming robots?
            </MotionText>
            <MotionButton
              size="lg"
              variant="gradient"
              onClick={onGetStarted}
              fontSize="lg"
              px={12}
              py={6}
              h="auto"
              whileHover={{ scale: 1.05 }}
              whileTap={{ scale: 0.95 }}
            >
              ▶️ Book Your Session Now
            </MotionButton>
          </VStack>
        </MotionBox>
      </VStack>

      {/* Robot Details Modal */}
      <Modal isOpen={isOpen} onClose={onClose} size="lg">
        <ModalOverlay backdropFilter="blur(10px)" />
        <ModalContent 
          bg="rgba(15, 23, 42, 0.95)" 
          color="white"
          border="1px solid rgba(255, 255, 255, 0.1)"
          borderRadius="2xl"
          backdropFilter="blur(20px)"
        >
          <ModalHeader>
            <HStack>
              <Text fontSize="3xl" filter="drop-shadow(0 0 10px rgba(56, 189, 248, 0.3))">
                {selectedRobot?.image}
              </Text>
              <Text fontSize="xl" fontWeight="bold">{selectedRobot?.name}</Text>
            </HStack>
          </ModalHeader>
          <ModalCloseButton />
          <ModalBody pb={6}>
            <VStack spacing={6} align="start">
              <Text color="gray.300" fontSize="lg" lineHeight="1.6">
                {selectedRobot?.description}
              </Text>
              
              <Box w="full">
                <Text fontWeight="bold" mb={4} fontSize="lg">Key Features:</Text>
                <VStack spacing={3} align="start">
                  {selectedRobot?.features.map((feature, index) => (
                    <HStack key={index}>
                      <Box color="success.400" fontSize="lg">✓</Box>
                      <Text>{feature}</Text>
                    </HStack>
                  ))}
                </VStack>
              </Box>

              <Box w="full" pt={4}>
                <MotionButton
                  variant="gradient"
                  onClick={() => {
                    onClose();
                    onGetStarted();
                  }}
                  w="full"
                  py={6}
                  h="auto"
                  whileHover={{ scale: 1.02 }}
                  whileTap={{ scale: 0.98 }}
                >
                  🚀 Start Programming This Robot
                </MotionButton>
              </Box>
            </VStack>
          </ModalBody>
        </ModalContent>
      </Modal>
    </Container>
  );
};

export default LandingPage;