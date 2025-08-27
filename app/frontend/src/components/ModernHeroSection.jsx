import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Container,
  SimpleGrid,
  Card,
  CardBody,
  Icon,
  Flex,
  Link,
  Image,
  Avatar,
  Badge,
  useBreakpointValue,
} from "@chakra-ui/react";
import { 
  FaRobot, 
  FaCode, 
  FaCog, 
  FaCalendarAlt, 
  FaArrowRight,
  FaPhone,
  FaPlay,
  FaGithub,
  FaLinkedin,
  FaUsers,
  FaBolt,
  FaShieldAlt,
  FaCogs
} from "react-icons/fa";
import { motion } from "framer-motion";
import { useState } from "react";

// Motion components
const MotionBox = motion(Box);
const MotionText = motion(Text);
const MotionVStack = motion(VStack);
const MotionHStack = motion(HStack);

const ModernHeroSection = ({ onGetStarted }) => {
  const heroHeight = useBreakpointValue({ base: "90vh", md: "100vh" });
  
  // Animation variants
  const containerVariants = {
    hidden: { opacity: 0 },
    visible: {
      opacity: 1,
      transition: {
        staggerChildren: 0.3,
        delayChildren: 0.2,
      },
    },
  };

  const itemVariants = {
    hidden: { opacity: 0, y: 50 },
    visible: {
      opacity: 1,
      y: 0,
      transition: {
        duration: 0.8,
        ease: "easeOut",
      },
    },
  };

  const floatingVariants = {
    floating: {
      y: [-10, 10, -10],
      transition: {
        duration: 4,
        repeat: Infinity,
        ease: "easeInOut",
      },
    },
  };

  return (
    <Box 
      minH="100vh"
      bg="white"
      color="gray.800"
      position="relative"
      overflow="hidden"
    >
      {/* Header Navigation */}
      <MotionBox
        position="absolute"
        top="0"
        left="0"
        right="0"
        zIndex="10"
        initial={{ opacity: 0, y: -50 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.8, delay: 0.5 }}
      >
        <Container maxW="7xl" py={6}>
          <Flex justify="space-between" align="center">
            {/* Logo */}
            <HStack spacing={3}>
              <MotionBox
                animate={{
                  rotate: [0, 360],
                }}
                transition={{
                  duration: 20,
                  repeat: Infinity,
                  ease: "linear",
                }}
              >
                <Icon as={FaRobot} fontSize="2xl" color="cyan.400" />
              </MotionBox>
              <Text fontSize="xl" fontWeight="bold" color="white">
                RobotConsole
              </Text>
            </HStack>

            {/* Navigation Menu */}
            <HStack spacing={8} display={{ base: "none", md: "flex" }}>
              <Link color="blue.100" _hover={{ color: "white" }} fontSize="md" fontWeight="medium">
                Features
              </Link>
              <Link color="blue.100" _hover={{ color: "white" }} fontSize="md" fontWeight="medium">
                Pricing
              </Link>
              <Link color="blue.100" _hover={{ color: "white" }} fontSize="md" fontWeight="medium">
                Docs
              </Link>
              <Link color="blue.100" _hover={{ color: "white" }} fontSize="md" fontWeight="medium">
                Support
              </Link>
              <Button
                size="sm"
                bgGradient="linear(135deg, cyan.400, blue.500)"
                color="white"
                _hover={{
                  bgGradient: "linear(135deg, cyan.300, blue.400)",
                  transform: "translateY(-1px)",
                }}
                onClick={onGetStarted}
              >
                Get Started
              </Button>
            </HStack>
          </Flex>
        </Container>
      </MotionBox>

      {/* Main Hero Section */}
      <Box
        position="relative"
        minH={heroHeight}
        bgGradient="linear(135deg, blue.900 0%, blue.800 25%, blue.700 50%, blue.600 75%, blue.500 100%)"
        overflow="hidden"
      >
        {/* Enhanced Background Pattern */}
        <MotionBox
          position="absolute"
          top="0"
          left="0"
          right="0"
          bottom="0"
          opacity="0.1"
          backgroundImage="url('data:image/svg+xml,<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 200 200\"><path d=\"M20 20h40v40h-40zM80 20h40v40h-40zM140 20h40v40h-40zM20 80h40v40h-40zM80 80h40v40h-40zM140 80h40v40h-40zM20 140h40v40h-40zM80 140h40v40h-40zM140 140h40v40h-40z\" fill=\"none\" stroke=\"%23ffffff\" stroke-width=\"2\"/><circle cx=\"40\" cy=\"40\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"100\" cy=\"40\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"160\" cy=\"40\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"40\" cy=\"100\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"100\" cy=\"100\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"160\" cy=\"100\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"40\" cy=\"160\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"100\" cy=\"160\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"160\" cy=\"160\" r=\"5\" fill=\"%23ffffff\"/></svg>')"
          backgroundSize="120px 120px"
          zIndex="0"
          animate={{
            backgroundPosition: ["0px 0px", "120px 120px"],
          }}
          transition={{
            duration: 30,
            repeat: Infinity,
            ease: "linear",
          }}
        />
        
        {/* Floating geometric elements */}
        <MotionBox
          position="absolute"
          top="20%"
          left="10%"
          w="60px"
          h="60px"
          borderRadius="full"
          bgGradient="radial(circle, cyan.400 0%, transparent 70%)"
          opacity="0.3"
          variants={floatingVariants}
          animate="floating"
        />
        
        <MotionBox
          position="absolute"
          top="60%"
          right="15%"
          w="40px"
          h="40px"
          bg="blue.400"
          opacity="0.2"
          transform="rotate(45deg)"
          variants={floatingVariants}
          animate="floating"
          transition={{ delay: 2 }}
        />

        <Container maxW="7xl" position="relative" zIndex="1">
          <MotionVStack
            minH={heroHeight}
            justify="center"
            align="center"
            spacing={12}
            py={20}
            variants={containerVariants}
            initial="hidden"
            animate="visible"
          >
            <MotionHStack
              spacing={20}
              align="center"
              w="full"
              maxW="6xl"
              flexDir={{ base: "column", lg: "row" }}
              variants={itemVariants}
            >
              {/* Left Side - Enhanced Text Content */}
              <MotionVStack
                align={{ base: "center", lg: "start" }}
                spacing={8}
                flex="1"
                textAlign={{ base: "center", lg: "left" }}
                variants={itemVariants}
              >
                <VStack spacing={6} align={{ base: "center", lg: "start" }}>
                  {/* Badge */}
                  <MotionBox
                    initial={{ opacity: 0, scale: 0.8 }}
                    animate={{ opacity: 1, scale: 1 }}
                    transition={{ delay: 1, duration: 0.6 }}
                  >
                    <Badge
                      colorScheme="cyan"
                      px={4}
                      py={2}
                      borderRadius="full"
                      fontSize="sm"
                      fontWeight="bold"
                      bgGradient="linear(135deg, cyan.400, blue.500)"
                      color="white"
                    >
                      🚀 Next-Gen Robot Development
                    </Badge>
                  </MotionBox>

                  <MotionText
                    fontSize={{ base: "4xl", md: "5xl", lg: "6xl" }}
                    fontWeight="900"
                    color="white"
                    lineHeight="1.1"
                    textShadow="0 4px 20px rgba(0, 0, 0, 0.3)"
                    variants={itemVariants}
                  >
                    Professional{" "}
                    <MotionText 
                      as="span" 
                      color="cyan.300"
                      bgGradient="linear(45deg, cyan.300, cyan.500)"
                      bgClip="text"
                      animate={{
                        backgroundPosition: ["0% 50%", "100% 50%", "0% 50%"],
                      }}
                      transition={{
                        duration: 3,
                        repeat: Infinity,
                      }}
                    >
                      Robot Development
                    </MotionText>{" "}
                    Console
                  </MotionText>
                  
                  <MotionText
                    fontSize={{ base: "lg", md: "xl", lg: "2xl" }}
                    color="blue.100"
                    maxW="2xl"
                    lineHeight="1.6"
                    variants={itemVariants}
                  >
                    Book dedicated time slots for advanced robotics development. 
                    Access professional environments with{" "}
                    <Text as="span" color="cyan.300" fontWeight="bold">ROS</Text>, {" "}
                    <Text as="span" color="cyan.300" fontWeight="bold">Gazebo</Text>, and {" "}
                    <Text as="span" color="cyan.300" fontWeight="bold">real-time simulation</Text> {" "}
                    for TurtleBot navigation, robotic arm manipulation, and multi-robot coordination.
                  </MotionText>
                </VStack>
                
                <MotionHStack 
                  spacing={6} 
                  flexWrap="wrap" 
                  justify={{ base: "center", lg: "start" }}
                  variants={itemVariants}
                >
                  <MotionBox
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                  >
                    <Button
                      size="lg"
                      px={8}
                      py={6}
                      fontSize="lg"
                      fontWeight="bold"
                      bgGradient="linear(135deg, cyan.400, blue.500)"
                      color="white"
                      boxShadow="0 8px 32px rgba(0, 255, 255, 0.3)"
                      _hover={{
                        bgGradient: "linear(135deg, cyan.300, blue.400)",
                        boxShadow: "0 12px 48px rgba(0, 255, 255, 0.4)",
                        transform: "translateY(-2px)",
                      }}
                      leftIcon={<FaCalendarAlt />}
                      onClick={onGetStarted}
                    >
                      Book Development Session
                    </Button>
                  </MotionBox>
                  
                  <MotionBox
                    whileHover={{ scale: 1.05 }}
                    whileTap={{ scale: 0.95 }}
                  >
                    <Button
                      size="lg"
                      px={8}
                      py={6}
                      fontSize="lg"
                      variant="outline"
                      borderColor="cyan.300"
                      borderWidth="2px"
                      color="white"
                      _hover={{
                        bg: "whiteAlpha.200",
                        borderColor: "cyan.200",
                        transform: "translateY(-2px)",
                      }}
                      leftIcon={<FaPlay />}
                    >
                      View Available Slots
                    </Button>
                  </MotionBox>
                </MotionHStack>

                {/* Stats */}
                <MotionHStack 
                  spacing={8}
                  pt={4}
                  variants={itemVariants}
                  justify={{ base: "center", lg: "start" }}
                >
                  <VStack spacing={1} align="center">
                    <Text fontSize="2xl" fontWeight="bold" color="cyan.300">24/7</Text>
                    <Text fontSize="sm" color="blue.200">Console Access</Text>
                  </VStack>
                  <VStack spacing={1} align="center">
                    <Text fontSize="2xl" fontWeight="bold" color="cyan.300">50+</Text>
                    <Text fontSize="sm" color="blue.200">Robot Projects</Text>
                  </VStack>
                  <VStack spacing={1} align="center">
                    <Text fontSize="2xl" fontWeight="bold" color="cyan.300">99.9%</Text>
                    <Text fontSize="sm" color="blue.200">Uptime</Text>
                  </VStack>
                </MotionHStack>
              </MotionVStack>
              
              {/* Right Side - Enhanced 3D Robot Illustration */}
              <MotionBox
                flex="1"
                display="flex"
                justify="center"
                align="center"
                position="relative"
                variants={itemVariants}
              >
                <MotionBox
                  position="relative"
                  w={{ base: "300px", md: "400px", lg: "500px" }}
                  h={{ base: "300px", md: "400px", lg: "500px" }}
                  display="flex"
                  align="center"
                  justify="center"
                  initial={{ scale: 0.8, opacity: 0 }}
                  animate={{ scale: 1, opacity: 1 }}
                  transition={{ delay: 0.8, duration: 1 }}
                >
                  {/* Enhanced Glow Effects */}
                  <MotionBox
                    position="absolute"
                    w="400px"
                    h="400px"
                    bgGradient="radial(circle, cyan.400 0%, transparent 70%)"
                    opacity="0.4"
                    borderRadius="full"
                    filter="blur(60px)"
                    animate={{
                      scale: [1, 1.2, 1],
                      opacity: [0.4, 0.6, 0.4],
                    }}
                    transition={{
                      duration: 4,
                      repeat: Infinity,
                      ease: "easeInOut",
                    }}
                  />
                  
                  <MotionBox
                    position="absolute"
                    w="200px"
                    h="200px"
                    bgGradient="radial(circle, blue.400 0%, transparent 60%)"
                    opacity="0.3"
                    borderRadius="full"
                    filter="blur(30px)"
                    animate={{
                      scale: [1.2, 1, 1.2],
                      opacity: [0.3, 0.5, 0.3],
                    }}
                    transition={{
                      duration: 3,
                      repeat: Infinity,
                      ease: "easeInOut",
                      delay: 1,
                    }}
                  />
                  
                  {/* Robot Icon with enhanced effects */}
                  <MotionBox
                    animate={{
                      y: [-10, 10, -10],
                      rotateY: [0, 10, 0, -10, 0],
                    }}
                    transition={{
                      duration: 6,
                      repeat: Infinity,
                      ease: "easeInOut",
                    }}
                  >
                    <Icon
                      as={FaRobot}
                      fontSize={{ base: "200px", md: "250px", lg: "350px" }}
                      color="cyan.300"
                      filter="drop-shadow(0 15px 40px rgba(0, 255, 255, 0.6))"
                    />
                  </MotionBox>

                  {/* Floating elements around robot */}
                  <MotionBox
                    position="absolute"
                    top="20%"
                    left="10%"
                    animate={{
                      y: [-5, 5, -5],
                      x: [-2, 2, -2],
                    }}
                    transition={{
                      duration: 3,
                      repeat: Infinity,
                      ease: "easeInOut",
                    }}
                  >
                    <Icon as={FaCode} fontSize="2xl" color="cyan.200" opacity="0.7" />
                  </MotionBox>
                  
                  <MotionBox
                    position="absolute"
                    top="70%"
                    right="20%"
                    animate={{
                      y: [5, -5, 5],
                      x: [2, -2, 2],
                    }}
                    transition={{
                      duration: 4,
                      repeat: Infinity,
                      ease: "easeInOut",
                      delay: 1,
                    }}
                  >
                    <Icon as={FaCogs} fontSize="2xl" color="blue.200" opacity="0.7" />
                  </MotionBox>
                </MotionBox>
              </MotionBox>
            </MotionHStack>
          </MotionVStack>
        </Container>
      </Box>

      {/* Feature Showcase Section */}
      <MotionBox
        py={20}
        bg="gray.50"
        initial={{ opacity: 0 }}
        whileInView={{ opacity: 1 }}
        transition={{ duration: 0.8 }}
        viewport={{ once: true }}
      >
        <Container maxW="7xl">
          <MotionVStack spacing={12}>
            <VStack spacing={4} textAlign="center">
              <MotionText
                fontSize={{ base: "3xl", md: "4xl" }}
                fontWeight="bold"
                color="blue.900"
                initial={{ opacity: 0, y: 30 }}
                whileInView={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.8 }}
                viewport={{ once: true }}
              >
                Powerful Development Features
              </MotionText>
              <Text
                fontSize="xl"
                color="gray.600"
                maxW="2xl"
              >
                Everything you need for professional robot development in one integrated platform
              </Text>
            </VStack>

            <MotionBox
              initial={{ opacity: 0, y: 50 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8, delay: 0.2 }}
              viewport={{ once: true }}
            >
              <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={8}>
                {[
                  {
                    icon: FaCode,
                    title: "Advanced Code Editor",
                    description: "Full-featured IDE with syntax highlighting, debugging, and Git integration for robot programming.",
                  },
                  {
                    icon: FaCogs,
                    title: "Gazebo Simulation",
                    description: "Real-time 3D simulation environment for testing robot behaviors before hardware deployment.",
                  },
                  {
                    icon: FaCalendarAlt,
                    title: "Time Slot Booking",
                    description: "Reserve dedicated development sessions with guaranteed resource availability.",
                  },
                  {
                    icon: FaUsers,
                    title: "Multi-Robot Support",
                    description: "Simultaneous control of TurtleBot, robot arms, and manipulation systems in one session.",
                  },
                ].map((feature, index) => (
                  <MotionBox
                    key={index}
                    whileHover={{ y: -5 }}
                    transition={{ duration: 0.3 }}
                  >
                    <Card
                      h="full"
                      bg="white"
                      borderRadius="20px"
                      boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                      border="1px solid"
                      borderColor="blue.100"
                      _hover={{
                        borderColor: "cyan.300",
                        boxShadow: "0 15px 40px rgba(59, 130, 246, 0.2)",
                      }}
                      transition="all 0.3s ease"
                    >
                      <CardBody textAlign="center" py={8}>
                        <VStack spacing={4}>
                          <Box
                            w="60px"
                            h="60px"
                            bgGradient="linear(135deg, cyan.400, blue.500)"
                            borderRadius="20px"
                            display="flex"
                            align="center"
                            justify="center"
                          >
                            <Icon as={feature.icon} fontSize="2xl" color="white" />
                          </Box>
                          <Text fontSize="xl" fontWeight="bold" color="blue.900">
                            {feature.title}
                          </Text>
                          <Text color="gray.600" lineHeight="1.6">
                            {feature.description}
                          </Text>
                        </VStack>
                      </CardBody>
                    </Card>
                  </MotionBox>
                ))}
              </SimpleGrid>
            </MotionBox>
          </MotionVStack>
        </Container>
      </MotionBox>

      {/* Customer Logos Section */}
      <MotionBox
        py={16}
        bg="white"
        initial={{ opacity: 0 }}
        whileInView={{ opacity: 1 }}
        transition={{ duration: 0.8 }}
        viewport={{ once: true }}
      >
        <Container maxW="6xl">
          <VStack spacing={12}>
            <VStack spacing={4} textAlign="center">
              <Text
                fontSize="lg"
                color="gray.500"
                fontWeight="medium"
                letterSpacing="wider"
                textTransform="uppercase"
              >
                Trusted by Leading Organizations
              </Text>
            </VStack>

            <MotionBox
              initial={{ opacity: 0, y: 30 }}
              whileInView={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8 }}
              viewport={{ once: true }}
            >
              <SimpleGrid columns={{ base: 2, md: 4, lg: 6 }} spacing={8} alignItems="center">
                {[
                  { name: "MIT", color: "gray.400" },
                  { name: "Stanford", color: "gray.400" },
                  { name: "Carnegie Mellon", color: "gray.400" },
                  { name: "UC Berkeley", color: "gray.400" },
                  { name: "Georgia Tech", color: "gray.400" },
                  { name: "ETH Zurich", color: "gray.400" },
                ].map((org, index) => (
                  <MotionBox
                    key={index}
                    textAlign="center"
                    whileHover={{ scale: 1.1 }}
                    transition={{ duration: 0.3 }}
                  >
                    <Text
                      fontSize="lg"
                      fontWeight="bold"
                      color={org.color}
                      _hover={{ color: "blue.500" }}
                      transition="color 0.3s ease"
                    >
                      {org.name}
                    </Text>
                  </MotionBox>
                ))}
              </SimpleGrid>
            </MotionBox>
          </VStack>
        </Container>
      </MotionBox>
    </Box>
  );
};

export default ModernHeroSection;