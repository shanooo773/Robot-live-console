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
  keyframes,
} from "@chakra-ui/react";
import { useState } from "react";
import { motion } from "framer-motion";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);

// Animation keyframes
const float = keyframes`
  0% { transform: translateY(0px); }
  50% { transform: translateY(-10px); }
  100% { transform: translateY(0px); }
`;

const glow = keyframes`
  0% { box-shadow: 0 0 20px rgba(102, 126, 234, 0.3); }
  50% { box-shadow: 0 0 30px rgba(102, 126, 234, 0.6); }
  100% { box-shadow: 0 0 20px rgba(102, 126, 234, 0.3); }
`;

const fadeInUp = {
  hidden: { opacity: 0, y: 60 },
  visible: { 
    opacity: 1, 
    y: 0,
    transition: { duration: 0.8, ease: "easeOut" }
  }
};

const staggerContainer = {
  hidden: { opacity: 0 },
  visible: {
    opacity: 1,
    transition: {
      staggerChildren: 0.2,
      delayChildren: 0.3
    }
  }
};

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
    <Container maxW="7xl" py={20}>
      <MotionVStack 
        spacing={20} 
        align="center"
        initial="hidden"
        animate="visible"
        variants={staggerContainer}
      >
        {/* Hero Section with Enhanced Animation */}
        <MotionVStack 
          spacing={8} 
          textAlign="center"
          variants={fadeInUp}
        >
          <MotionBox
            fontSize="7xl"
            animation={`${float} 3s ease-in-out infinite`}
            variants={fadeInUp}
          >
            🤖
          </MotionBox>
          
          <VStack spacing={4}>
            <Text 
              fontSize={{ base: "4xl", md: "6xl" }} 
              fontWeight="bold" 
              bgGradient="linear(to-r, #667eea, #764ba2, #f093fb)"
              bgClip="text"
              lineHeight="shorter"
            >
              Robot Live Console
            </Text>
            <Text 
              fontSize={{ base: "lg", md: "xl" }} 
              color="gray.300" 
              maxW="2xl"
              lineHeight="relaxed"
            >
              Experience the future of robotics education with real-time programming, 
              simulation, and interactive learning in a live environment.
            </Text>
          </VStack>
          
          <MotionBox variants={fadeInUp}>
            <Button
              size="lg"
              variant="gradient"
              onClick={onGetStarted}
              fontSize="lg"
              px={12}
              py={8}
              h="auto"
              borderRadius="xl"
              animation={`${glow} 2s ease-in-out infinite`}
            >
              Start Your Robot Journey
            </Button>
          </MotionBox>
        </MotionVStack>

        {/* How It Works Section with Enhanced Design */}
        <MotionVStack spacing={12} w="full" variants={fadeInUp}>
          <Text 
            fontSize={{ base: "3xl", md: "4xl" }} 
            fontWeight="bold" 
            bgGradient="linear(to-r, #43e97b, #38f9d7)"
            bgClip="text"
          >
            How It Works
          </Text>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            <MotionCard 
              variant="floating"
              initial={{ opacity: 0, y: 50 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.2, duration: 0.6 }}
              animation={`${float} 4s ease-in-out infinite`}
              animationDelay="0s"
            >
              <CardBody textAlign="center" p={8}>
                <Text fontSize="5xl" mb={6}>📅</Text>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                  1. Book a Time Slot
                </Text>
                <Text color="gray.300" lineHeight="relaxed">
                  Choose an available time slot that fits your schedule and select your preferred robot
                </Text>
              </CardBody>
            </MotionCard>

            <MotionCard 
              variant="floating"
              initial={{ opacity: 0, y: 50 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.4, duration: 0.6 }}
              animation={`${float} 4s ease-in-out infinite`}
              animationDelay="1.5s"
            >
              <CardBody textAlign="center" p={8}>
                <Text fontSize="5xl" mb={6}>💻</Text>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                  2. Write Python Code
                </Text>
                <Text color="gray.300" lineHeight="relaxed">
                  Use our advanced Monaco editor to write ROS Python code with syntax highlighting
                </Text>
              </CardBody>
            </MotionCard>

            <MotionCard 
              variant="floating"
              initial={{ opacity: 0, y: 50 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ delay: 0.6, duration: 0.6 }}
              animation={`${float} 4s ease-in-out infinite`}
              animationDelay="3s"
            >
              <CardBody textAlign="center" p={8}>
                <Text fontSize="5xl" mb={6}>🎥</Text>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                  3. See Live Results
                </Text>
                <Text color="gray.300" lineHeight="relaxed">
                  Watch your robot perform tasks in real-time simulation with instant feedback
                </Text>
              </CardBody>
            </MotionCard>
          </SimpleGrid>
        </MotionVStack>

        {/* Available Robots Section with Enhanced Design */}
        <MotionVStack spacing={12} w="full" variants={fadeInUp}>
          <Text 
            fontSize={{ base: "3xl", md: "4xl" }} 
            fontWeight="bold" 
            bgGradient="linear(to-r, #f093fb, #f5576c)"
            bgClip="text"
          >
            Available Robots
          </Text>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            {robotTypes.map((robot, index) => (
              <MotionCard 
                key={robot.id} 
                variant="floating"
                cursor="pointer"
                onClick={() => handleRobotClick(robot)}
                initial={{ opacity: 0, scale: 0.8 }}
                animate={{ opacity: 1, scale: 1 }}
                transition={{ delay: index * 0.2, duration: 0.6 }}
                whileHover={{ 
                  scale: 1.05,
                  rotate: 2,
                  transition: { duration: 0.3 }
                }}
                whileTap={{ scale: 0.95 }}
              >
                <CardBody textAlign="center" p={8}>
                  <Text fontSize="6xl" mb={6} animation={`${float} 3s ease-in-out infinite`} 
                        style={{ animationDelay: `${index * 0.5}s` }}>
                    {robot.image}
                  </Text>
                  <Text fontSize="xl" fontWeight="bold" color="white" mb={4}>
                    {robot.name}
                  </Text>
                  <Text color="gray.300" mb={6} lineHeight="relaxed">
                    {robot.description}
                  </Text>
                  
                  <VStack spacing={3}>
                    {robot.features.map((feature, featureIndex) => (
                      <Badge 
                        key={featureIndex} 
                        variant="glass" 
                        borderRadius="full"
                        px={3}
                        py={1}
                      >
                        {feature}
                      </Badge>
                    ))}
                  </VStack>
                </CardBody>
              </MotionCard>
            ))}
          </SimpleGrid>
        </MotionVStack>

        {/* Call to Action Section */}
        <MotionVStack 
          spacing={8} 
          textAlign="center" 
          py={16}
          variants={fadeInUp}
        >
          <Text 
            fontSize={{ base: "2xl", md: "3xl" }} 
            fontWeight="bold" 
            bgGradient="linear(to-r, #43e97b, #38f9d7)"
            bgClip="text"
          >
            Ready to start programming robots?
          </Text>
          <Text color="gray.300" maxW="lg" lineHeight="relaxed">
            Join thousands of students and professionals learning robotics through hands-on programming
          </Text>
          <Button
            size="lg"
            variant="success"
            onClick={onGetStarted}
            fontSize="lg"
            px={12}
            py={8}
            h="auto"
            borderRadius="xl"
          >
            Book Your Session Now
          </Button>
        </MotionVStack>
      </MotionVStack>

      {/* Enhanced Robot Details Modal */}
      <Modal isOpen={isOpen} onClose={onClose} size="lg">
        <ModalOverlay backdropFilter="blur(10px)" />
        <MotionCard 
          as={ModalContent} 
          variant="glass"
          initial={{ opacity: 0, scale: 0.8, y: 50 }}
          animate={{ opacity: 1, scale: 1, y: 0 }}
          transition={{ duration: 0.4 }}
        >
          <ModalHeader>
            <HStack>
              <Text fontSize="4xl">{selectedRobot?.image}</Text>
              <Text 
                bgGradient="linear(to-r, #667eea, #764ba2)"
                bgClip="text"
                fontWeight="bold"
              >
                {selectedRobot?.name}
              </Text>
            </HStack>
          </ModalHeader>
          <ModalCloseButton />
          <ModalBody pb={8}>
            <VStack spacing={6} align="start">
              <Text color="gray.300" lineHeight="relaxed">
                {selectedRobot?.description}
              </Text>
              
              <Box>
                <Text fontWeight="bold" mb={4} color="white">
                  Key Features:
                </Text>
                <VStack spacing={3} align="start">
                  {selectedRobot?.features.map((feature, index) => (
                    <HStack key={index}>
                      <Text color="green.400" fontSize="lg">✓</Text>
                      <Text color="gray.300">{feature}</Text>
                    </HStack>
                  ))}
                </VStack>
              </Box>

              <Box w="full" pt={4}>
                <Button
                  variant="gradient"
                  onClick={() => {
                    onClose();
                    onGetStarted();
                  }}
                  w="full"
                  size="lg"
                  borderRadius="xl"
                >
                  Start Programming This Robot
                </Button>
              </Box>
            </VStack>
          </ModalBody>
        </MotionCard>
      </Modal>
    </Container>
  );
};

export default LandingPage;