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
  CardHeader,
  Image,
  Badge,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  Accordion,
  AccordionItem,
  AccordionButton,
  AccordionPanel,
  AccordionIcon,
  Icon,
  Input,
  Textarea,
  FormControl,
  FormLabel,
  Flex,
  Avatar,
  Divider,
  Link,
} from "@chakra-ui/react";
import { 
  FaRobot, 
  FaBrain, 
  FaGraduationCap, 
  FaChartLine, 
  FaCogs, 
  FaLightbulb,
  FaShieldAlt,
  FaRocket,
  FaPhone,
  FaEnvelope,
  FaMapMarkerAlt,
  FaFacebook,
  FaTwitter,
  FaLinkedin,
  FaGithub,
  FaStar,
  FaArrowRight,
  FaPlay,
  FaQuoteLeft,
  FaUsers,
  FaAward,
  FaGlobe,
  FaHandshake
} from "react-icons/fa";

const LandingPage = ({ onGetStarted }) => {
  return (
    <Box 
      minH="100vh"
      bg="white"
      color="gray.800"
      position="relative"
      overflow="hidden"
    >
      {/* Hero Section */}
      <Box
        position="relative"
        minH="100vh"
        bgGradient="linear(135deg, blue.900 0%, blue.800 25%, blue.700 50%, blue.600 75%, blue.500 100%)"
        overflow="hidden"
      >
        {/* Background Circuit Pattern */}
        <Box
          position="absolute"
          top="0"
          left="0"
          right="0"
          bottom="0"
          opacity="0.1"
          backgroundImage="url('data:image/svg+xml,<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 100 100\"><path d=\"M10 10h20v20h-20zM40 10h20v20h-20zM70 10h20v20h-20zM10 40h20v20h-20zM40 40h20v20h-20zM70 40h20v20h-20zM10 70h20v20h-20zM40 70h20v20h-20zM70 70h20v20h-20z\" fill=\"none\" stroke=\"%23ffffff\" stroke-width=\"1\"/></svg>')"
          backgroundSize="60px 60px"
          zIndex="0"
        />
        
        <Container maxW="7xl" position="relative" zIndex="1">
          <VStack
            minH="100vh"
            justify="center"
            align="center"
            spacing={12}
            py={20}
          >
            <HStack
              spacing={20}
              align="center"
              w="full"
              maxW="6xl"
              flexDir={{ base: "column", lg: "row" }}
            >
              {/* Left Side - Text Content */}
              <VStack
                align={{ base: "center", lg: "start" }}
                spacing={8}
                flex="1"
                textAlign={{ base: "center", lg: "left" }}
              >
                <VStack spacing={4} align={{ base: "center", lg: "start" }}>
                  <Text
                    fontSize={{ base: "4xl", md: "5xl", lg: "6xl" }}
                    fontWeight="900"
                    color="white"
                    lineHeight="1.1"
                    textShadow="0 4px 20px rgba(0, 0, 0, 0.3)"
                  >
                    Book Your{" "}
                    <Text as="span" color="cyan.300">
                      Robot Development
                    </Text>{" "}
                    Console Session
                  </Text>
                  
                  <Text
                    fontSize={{ base: "lg", md: "xl" }}
                    color="blue.100"
                    maxW="2xl"
                    lineHeight="1.6"
                  >
                    Reserve dedicated time slots to code, test, and simulate robotics projects. 
                    Access professional development environments with ROS, Gazebo, and real-time 
                    video feedback for TurtleBot, Robot Arms, and Manipulation systems.
                  </Text>
                </VStack>
                
                <HStack spacing={6} flexWrap="wrap" justify={{ base: "center", lg: "start" }}>
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
                    leftIcon={<FaArrowRight />}
                    onClick={onGetStarted}
                  >
                    Book Development Session
                  </Button>
                  
                  <Button
                    size="lg"
                    px={8}
                    py={6}
                    fontSize="lg"
                    variant="outline"
                    borderColor="cyan.300"
                    color="white"
                    _hover={{
                      bg: "whiteAlpha.200",
                      borderColor: "cyan.200",
                      transform: "translateY(-2px)",
                    }}
                    leftIcon={<FaPhone />}
                  >
                    View Available Slots
                  </Button>
                </HStack>
              </VStack>
              
              {/* Right Side - 3D Robot Illustration */}
              <Box
                flex="1"
                display="flex"
                justify="center"
                align="center"
                position="relative"
              >
                <Box
                  position="relative"
                  w="400px"
                  h="400px"
                  display="flex"
                  align="center"
                  justify="center"
                >
                  {/* Glow Effect */}
                  <Box
                    position="absolute"
                    w="300px"
                    h="300px"
                    bgGradient="radial(circle, cyan.400 0%, transparent 70%)"
                    opacity="0.3"
                    borderRadius="full"
                    filter="blur(40px)"
                    animation="pulse 3s infinite"
                  />
                  
                  {/* Robot Icon */}
                  <Icon
                    as={FaRobot}
                    fontSize="300px"
                    color="cyan.300"
                    filter="drop-shadow(0 10px 30px rgba(0, 255, 255, 0.5))"
                    animation="float 6s ease-in-out infinite"
                  />
                </Box>
              </Box>
            </HStack>
          </VStack>
        </Container>
      </Box>

      {/* About Section */}
      <Container maxW="7xl" py={20}>
        <HStack
          spacing={20}
          align="center"
          w="full"
          flexDir={{ base: "column", lg: "row" }}
        >
          {/* Left Side - Image */}
          <Box
            flex="1"
            display="flex"
            justify="center"
            align="center"
            position="relative"
          >
            <Box
              w="350px"
              h="350px"
              bgGradient="linear(135deg, blue.500, cyan.400)"
              borderRadius="20px"
              display="flex"
              align="center"
              justify="center"
              position="relative"
              boxShadow="0 20px 60px rgba(59, 130, 246, 0.3)"
            >
              <Icon
                as={FaHandshake}
                fontSize="200px"
                color="white"
                filter="drop-shadow(0 5px 15px rgba(0, 0, 0, 0.2))"
              />
            </Box>
          </Box>
          
          {/* Right Side - Text Content */}
          <VStack
            align="start"
            spacing={8}
            flex="1"
            textAlign={{ base: "center", lg: "left" }}
          >
            <VStack spacing={4} align={{ base: "center", lg: "start" }}>
              <Text
                fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }}
                fontWeight="900"
                color="blue.900"
                lineHeight="1.2"
              >
                Dedicated Development{" "}
                <Text as="span" color="cyan.500">
                  Environments
                </Text>
              </Text>
              
              <Text
                fontSize="lg"
                color="gray.600"
                lineHeight="1.6"
                maxW="xl"
              >
                Access professional-grade robotics development environments through our 
                time-slot booking system. Code, test, and iterate on your robot projects 
                with dedicated computing resources and simulation capabilities.
              </Text>
            </VStack>
            
            {/* Stats/Features */}
            <SimpleGrid columns={{ base: 2, md: 2 }} spacing={6} w="full">
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="15px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 15px 40px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-5px)",
                }}
                transition="all 0.3s ease"
              >
                <CardBody textAlign="center" py={6}>
                  <Icon as={FaUsers} fontSize="3xl" color="cyan.500" mb={3} />
                  <Stat>
                    <StatNumber fontSize="2xl" color="blue.900" fontWeight="bold">
                      24/7
                    </StatNumber>
                    <StatLabel color="gray.600" fontSize="sm">
                      Console Access
                    </StatLabel>
                  </Stat>
                </CardBody>
              </Card>
              
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="15px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 15px 40px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-5px)",
                }}
                transition="all 0.3s ease"
              >
                <CardBody textAlign="center" py={6}>
                  <Icon as={FaAward} fontSize="3xl" color="cyan.500" mb={3} />
                  <Stat>
                    <StatNumber fontSize="2xl" color="blue.900" fontWeight="bold">
                      3
                    </StatNumber>
                    <StatLabel color="gray.600" fontSize="sm">
                      Robot Types
                    </StatLabel>
                  </Stat>
                </CardBody>
              </Card>
              
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="15px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 15px 40px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-5px)",
                }}
                transition="all 0.3s ease"
              >
                <CardBody textAlign="center" py={6}>
                  <Icon as={FaGlobe} fontSize="3xl" color="cyan.500" mb={3} />
                  <Stat>
                    <StatNumber fontSize="2xl" color="blue.900" fontWeight="bold">
                      ROS+Gazebo
                    </StatNumber>
                    <StatLabel color="gray.600" fontSize="sm">
                      Simulation Ready
                    </StatLabel>
                  </Stat>
                </CardBody>
              </Card>
              
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="15px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 15px 40px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-5px)",
                }}
                transition="all 0.3s ease"
              >
                <CardBody textAlign="center" py={6}>
                  <Icon as={FaShieldAlt} fontSize="3xl" color="cyan.500" mb={3} />
                  <Stat>
                    <StatNumber fontSize="2xl" color="blue.900" fontWeight="bold">
                      Real-time
                    </StatNumber>
                    <StatLabel color="gray.600" fontSize="sm">
                      Video Feedback
                    </StatLabel>
                  </Stat>
                </CardBody>
              </Card>
            </SimpleGrid>
          </VStack>
        </HStack>
      </Container>

      {/* Services Section */}
      <Box bg="gray.50" py={20}>
        <Container maxW="7xl">
          <VStack spacing={16} w="full">
            <VStack spacing={6} textAlign="center">
              <Text
                fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }}
                fontWeight="900"
                color="blue.900"
                lineHeight="1.2"
                maxW="4xl"
              >
                Professional Development{" "}
                <Text as="span" color="cyan.500">
                  Console Features
                </Text>
              </Text>
              <Text
                fontSize="lg"
                color="gray.600"
                maxW="2xl"
                lineHeight="1.6"
              >
                Book time slots and access dedicated robotics development environments with all the tools you need
              </Text>
            </VStack>
            
            <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={8} w="full">
              {/* Robotic Automation */}
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="20px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-10px)",
                }}
                transition="all 0.3s ease"
                overflow="hidden"
              >
                <CardBody textAlign="center" py={10}>
                  <Box
                    w="80px"
                    h="80px"
                    bgGradient="linear(135deg, cyan.400, blue.500)"
                    borderRadius="20px"
                    display="flex"
                    align="center"
                    justify="center"
                    mx="auto"
                    mb={6}
                  >
                    <Icon as={FaCogs} fontSize="3xl" color="white" />
                  </Box>
                  
                  <Text fontSize="xl" fontWeight="bold" color="blue.900" mb={4}>
                    Code Editor Console
                  </Text>
                  
                  <Text color="gray.600" lineHeight="1.6" mb={6}>
                    Professional Monaco editor with Python syntax highlighting, autocomplete, and ROS library support for robotics development.
                  </Text>
                  
                  <Button
                    size="sm"
                    variant="ghost"
                    color="cyan.500"
                    _hover={{ bg: "cyan.50" }}
                    rightIcon={<FaArrowRight />}
                  >
                    Learn More
                  </Button>
                </CardBody>
              </Card>
              
              {/* Machine Learning */}
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="20px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-10px)",
                }}
                transition="all 0.3s ease"
                overflow="hidden"
              >
                <CardBody textAlign="center" py={10}>
                  <Box
                    w="80px"
                    h="80px"
                    bgGradient="linear(135deg, cyan.400, blue.500)"
                    borderRadius="20px"
                    display="flex"
                    align="center"
                    justify="center"
                    mx="auto"
                    mb={6}
                  >
                    <Icon as={FaBrain} fontSize="3xl" color="white" />
                  </Box>
                  
                  <Text fontSize="xl" fontWeight="bold" color="blue.900" mb={4}>
                    Gazebo Simulation
                  </Text>
                  
                  <Text color="gray.600" lineHeight="1.6" mb={6}>
                    Test your robot code in realistic 3D environments with physics simulation and real-time video feedback of your robot's behavior.
                  </Text>
                  
                  <Button
                    size="sm"
                    variant="ghost"
                    color="cyan.500"
                    _hover={{ bg: "cyan.50" }}
                    rightIcon={<FaArrowRight />}
                  >
                    Learn More
                  </Button>
                </CardBody>
              </Card>
              
              {/* Education & Science */}
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="20px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-10px)",
                }}
                transition="all 0.3s ease"
                overflow="hidden"
              >
                <CardBody textAlign="center" py={10}>
                  <Box
                    w="80px"
                    h="80px"
                    bgGradient="linear(135deg, cyan.400, blue.500)"
                    borderRadius="20px"
                    display="flex"
                    align="center"
                    justify="center"
                    mx="auto"
                    mb={6}
                  >
                    <Icon as={FaGraduationCap} fontSize="3xl" color="white" />
                  </Box>
                  
                  <Text fontSize="xl" fontWeight="bold" color="blue.900" mb={4}>
                    Time Slot Booking
                  </Text>
                  
                  <Text color="gray.600" lineHeight="1.6" mb={6}>
                    Reserve dedicated development sessions with guaranteed access to computing resources and simulation environments.
                  </Text>
                  
                  <Button
                    size="sm"
                    variant="ghost"
                    color="cyan.500"
                    _hover={{ bg: "cyan.50" }}
                    rightIcon={<FaArrowRight />}
                  >
                    Learn More
                  </Button>
                </CardBody>
              </Card>
              
              {/* Predictive Analysis */}
              <Card
                bg="white"
                border="2px solid"
                borderColor="blue.100"
                borderRadius="20px"
                boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
                _hover={{
                  borderColor: "cyan.300",
                  boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                  transform: "translateY(-10px)",
                }}
                transition="all 0.3s ease"
                overflow="hidden"
              >
                <CardBody textAlign="center" py={10}>
                  <Box
                    w="80px"
                    h="80px"
                    bgGradient="linear(135deg, cyan.400, blue.500)"
                    borderRadius="20px"
                    display="flex"
                    align="center"
                    justify="center"
                    mx="auto"
                    mb={6}
                  >
                    <Icon as={FaChartLine} fontSize="3xl" color="white" />
                  </Box>
                  
                  <Text fontSize="xl" fontWeight="bold" color="blue.900" mb={4}>
                    Multi-Robot Support
                  </Text>
                  
                  <Text color="gray.600" lineHeight="1.6" mb={6}>
                    Work with TurtleBot navigation, robot arm manipulation, and dexterous hand control in separate booked development sessions.
                  </Text>
                  
                  <Button
                    size="sm"
                    variant="ghost"
                    color="cyan.500"
                    _hover={{ bg: "cyan.50" }}
                    rightIcon={<FaArrowRight />}
                  >
                    Learn More
                  </Button>
                </CardBody>
              </Card>
            </SimpleGrid>
          </VStack>
        </Container>
      </Box>

      {/* Business Growth Section */}
      <Box
        position="relative"
        py={20}
        bgGradient="linear(135deg, blue.900 0%, blue.800 25%, blue.700 50%, blue.600 75%, blue.500 100%)"
        overflow="hidden"
      >
        {/* Background Pattern */}
        <Box
          position="absolute"
          top="0"
          left="0"
          right="0"
          bottom="0"
          opacity="0.05"
          backgroundImage="url('data:image/svg+xml,<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 100 100\"><polygon points=\"0,0 50,0 100,50 50,100 0,50\" fill=\"%23ffffff\"/></svg>')"
          backgroundSize="120px 120px"
          zIndex="0"
        />
        
        <Container maxW="7xl" position="relative" zIndex="1">
          <HStack
            spacing={20}
            align="center"
            w="full"
            flexDir={{ base: "column", lg: "row" }}
          >
            {/* Left Side - Robot with Tablet */}
            <Box
              flex="1"
              display="flex"
              justify="center"
              align="center"
              position="relative"
            >
              <Box
                position="relative"
                w="400px"
                h="400px"
                display="flex"
                align="center"
                justify="center"
              >
                {/* Glow Effect */}
                <Box
                  position="absolute"
                  w="300px"
                  h="300px"
                  bgGradient="radial(circle, cyan.400 0%, transparent 70%)"
                  opacity="0.2"
                  borderRadius="full"
                  filter="blur(50px)"
                  animation="pulse 4s infinite"
                />
                
                {/* Robot with Tablet Illustration */}
                <VStack spacing={4}>
                  <Icon
                    as={FaRobot}
                    fontSize="250px"
                    color="cyan.300"
                    filter="drop-shadow(0 10px 30px rgba(0, 255, 255, 0.3))"
                  />
                  <Box
                    w="120px"
                    h="80px"
                    bg="white"
                    borderRadius="10px"
                    display="flex"
                    align="center"
                    justify="center"
                    boxShadow="0 10px 30px rgba(0, 0, 0, 0.3)"
                    position="relative"
                    top="-50px"
                  >
                    <Icon as={FaChartLine} fontSize="2xl" color="blue.500" />
                  </Box>
                </VStack>
              </Box>
            </Box>
            
            {/* Right Side - Text Content */}
            <VStack
              align={{ base: "center", lg: "start" }}
              spacing={8}
              flex="1"
              textAlign={{ base: "center", lg: "left" }}
            >
              <VStack spacing={4} align={{ base: "center", lg: "start" }}>
                <Text
                  fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }}
                  fontWeight="900"
                  color="white"
                  lineHeight="1.2"
                  maxW="3xl"
                >
                  Complete Development{" "}
                  <Text as="span" color="cyan.300">
                    Workflow in Your Browser
                  </Text>
                </Text>
                
                <Text
                  fontSize="lg"
                  color="blue.100"
                  lineHeight="1.6"
                  maxW="2xl"
                >
                  Book your time slot, access the code editor, write ROS Python code, and see your robot 
                  come to life in Gazebo simulation. Complete robotics development workflow with 
                  real-time feedback and video output of your simulation results.
                </Text>
              </VStack>
              
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
                leftIcon={<FaPlay />}
                onClick={onGetStarted}
              >
                Start Development Session
              </Button>
            </VStack>
          </HStack>
        </Container>
      </Box>

      {/* Case Studies Section */}
      <Container maxW="7xl" py={20}>
        <VStack spacing={16} w="full">
          <VStack spacing={6} textAlign="center">
            <Text
              fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }}
              fontWeight="900"
              color="blue.900"
              lineHeight="1.2"
            >
              Development{" "}
              <Text as="span" color="cyan.500">
                Success Stories
              </Text>
            </Text>
            <Text
              fontSize="lg"
              color="gray.600"
              maxW="2xl"
              lineHeight="1.6"
            >
              See how developers are using our console booking system to build amazing robotics projects
            </Text>
          </VStack>
          
          <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={8} w="full">
            {/* Case Study 1 */}
            <Card
              bg="white"
              borderRadius="20px"
              overflow="hidden"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              _hover={{
                boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                transform: "translateY(-5px)",
              }}
              transition="all 0.3s ease"
              position="relative"
              cursor="pointer"
            >
              <Box
                h="200px"
                bgGradient="linear(135deg, blue.500, cyan.400)"
                display="flex"
                align="center"
                justify="center"
                position="relative"
                overflow="hidden"
              >
                <Icon as={FaCogs} fontSize="6xl" color="white" opacity="0.7" />
                <Box
                  position="absolute"
                  top="0"
                  left="0"
                  right="0"
                  bottom="0"
                  bg="blackAlpha.300"
                  display="flex"
                  align="center"
                  justify="center"
                  opacity="0"
                  _hover={{ opacity: "1" }}
                  transition="opacity 0.3s ease"
                >
                  <Text color="white" fontSize="lg" fontWeight="bold">
                    View Case Study
                  </Text>
                </Box>
              </Box>
              <CardBody p={6}>
                <Text fontSize="lg" fontWeight="bold" color="blue.900" mb={3}>
                  TurtleBot Navigation Project
                </Text>
                <Text color="gray.600" lineHeight="1.6">
                  Student developed autonomous navigation algorithm in weekly 2-hour booked sessions, iterating through code and Gazebo simulations.
                </Text>
              </CardBody>
            </Card>
            
            {/* Case Study 2 */}
            <Card
              bg="white"
              borderRadius="20px"
              overflow="hidden"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              _hover={{
                boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                transform: "translateY(-5px)",
              }}
              transition="all 0.3s ease"
              position="relative"
              cursor="pointer"
            >
              <Box
                h="200px"
                bgGradient="linear(135deg, cyan.500, blue.400)"
                display="flex"
                align="center"
                justify="center"
                position="relative"
                overflow="hidden"
              >
                <Icon as={FaBrain} fontSize="6xl" color="white" opacity="0.7" />
                <Box
                  position="absolute"
                  top="0"
                  left="0"
                  right="0"
                  bottom="0"
                  bg="blackAlpha.300"
                  display="flex"
                  align="center"
                  justify="center"
                  opacity="0"
                  _hover={{ opacity: "1" }}
                  transition="opacity 0.3s ease"
                >
                  <Text color="white" fontSize="lg" fontWeight="bold">
                    View Case Study
                  </Text>
                </Box>
              </Box>
              <CardBody p={6}>
                <Text fontSize="lg" fontWeight="bold" color="blue.900" mb={3}>
                  Robot Arm Pick & Place
                </Text>
                <Text color="gray.600" lineHeight="1.6">
                  Research team prototyped manipulation tasks using booked development slots, testing complex grasping algorithms in simulation.
                </Text>
              </CardBody>
            </Card>
            
            {/* Case Study 3 */}
            <Card
              bg="white"
              borderRadius="20px"
              overflow="hidden"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              _hover={{
                boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                transform: "translateY(-5px)",
              }}
              transition="all 0.3s ease"
              position="relative"
              cursor="pointer"
            >
              <Box
                h="200px"
                bgGradient="linear(135deg, blue.400, cyan.500)"
                display="flex"
                align="center"
                justify="center"
                position="relative"
                overflow="hidden"
              >
                <Icon as={FaChartLine} fontSize="6xl" color="white" opacity="0.7" />
                <Box
                  position="absolute"
                  top="0"
                  left="0"
                  right="0"
                  bottom="0"
                  bg="blackAlpha.300"
                  display="flex"
                  align="center"
                  justify="center"
                  opacity="0"
                  _hover={{ opacity: "1" }}
                  transition="opacity 0.3s ease"
                >
                  <Text color="white" fontSize="lg" fontWeight="bold">
                    View Case Study
                  </Text>
                </Box>
              </Box>
              <CardBody p={6}>
                <Text fontSize="lg" fontWeight="bold" color="blue.900" mb={3}>
                  Multi-Robot Coordination
                </Text>
                <Text color="gray.600" lineHeight="1.6">
                  Startup team built swarm robotics demo by booking extended sessions and collaborating through shared development environments.
                </Text>
              </CardBody>
            </Card>
            
            {/* Case Study 4 */}
            <Card
              bg="white"
              borderRadius="20px"
              overflow="hidden"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              _hover={{
                boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                transform: "translateY(-5px)",
              }}
              transition="all 0.3s ease"
              position="relative"
              cursor="pointer"
            >
              <Box
                h="200px"
                bgGradient="linear(135deg, cyan.400, blue.600)"
                display="flex"
                align="center"
                justify="center"
                position="relative"
                overflow="hidden"
              >
                <Icon as={FaGraduationCap} fontSize="6xl" color="white" opacity="0.7" />
                <Box
                  position="absolute"
                  top="0"
                  left="0"
                  right="0"
                  bottom="0"
                  bg="blackAlpha.300"
                  display="flex"
                  align="center"
                  justify="center"
                  opacity="0"
                  _hover={{ opacity: "1" }}
                  transition="opacity 0.3s ease"
                >
                  <Text color="white" fontSize="lg" fontWeight="bold">
                    View Case Study
                  </Text>
                </Box>
              </Box>
              <CardBody p={6}>
                <Text fontSize="lg" fontWeight="bold" color="blue.900" mb={3}>
                  Educational Technology Innovation
                </Text>
                <Text color="gray.600" lineHeight="1.6">
                  Revolutionizing online learning with AI tutors that improved student engagement by 250%.
                </Text>
              </CardBody>
            </Card>
            
            {/* Case Study 5 */}
            <Card
              bg="white"
              borderRadius="20px"
              overflow="hidden"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              _hover={{
                boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                transform: "translateY(-5px)",
              }}
              transition="all 0.3s ease"
              position="relative"
              cursor="pointer"
            >
              <Box
                h="200px"
                bgGradient="linear(135deg, blue.600, cyan.400)"
                display="flex"
                align="center"
                justify="center"
                position="relative"
                overflow="hidden"
              >
                <Icon as={FaLightbulb} fontSize="6xl" color="white" opacity="0.7" />
                <Box
                  position="absolute"
                  top="0"
                  left="0"
                  right="0"
                  bottom="0"
                  bg="blackAlpha.300"
                  display="flex"
                  align="center"
                  justify="center"
                  opacity="0"
                  _hover={{ opacity: "1" }}
                  transition="opacity 0.3s ease"
                >
                  <Text color="white" fontSize="lg" fontWeight="bold">
                    View Case Study
                  </Text>
                </Box>
              </Box>
              <CardBody p={6}>
                <Text fontSize="lg" fontWeight="bold" color="blue.900" mb={3}>
                  Smart City Infrastructure
                </Text>
                <Text color="gray.600" lineHeight="1.6">
                  Building intelligent urban systems that reduced energy consumption by 40% using IoT and AI.
                </Text>
              </CardBody>
            </Card>
            
            {/* Case Study 6 */}
            <Card
              bg="white"
              borderRadius="20px"
              overflow="hidden"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              _hover={{
                boxShadow: "0 20px 50px rgba(59, 130, 246, 0.2)",
                transform: "translateY(-5px)",
              }}
              transition="all 0.3s ease"
              position="relative"
              cursor="pointer"
            >
              <Box
                h="200px"
                bgGradient="linear(135deg, cyan.600, blue.400)"
                display="flex"
                align="center"
                justify="center"
                position="relative"
                overflow="hidden"
              >
                <Icon as={FaShieldAlt} fontSize="6xl" color="white" opacity="0.7" />
                <Box
                  position="absolute"
                  top="0"
                  left="0"
                  right="0"
                  bottom="0"
                  bg="blackAlpha.300"
                  display="flex"
                  align="center"
                  justify="center"
                  opacity="0"
                  _hover={{ opacity: "1" }}
                  transition="opacity 0.3s ease"
                >
                  <Text color="white" fontSize="lg" fontWeight="bold">
                    View Case Study
                  </Text>
                </Box>
              </Box>
              <CardBody p={6}>
                <Text fontSize="lg" fontWeight="bold" color="blue.900" mb={3}>
                  Cybersecurity AI Defense
                </Text>
                <Text color="gray.600" lineHeight="1.6">
                  Protecting enterprise networks with AI-powered threat detection that stopped 99.9% of attacks.
                </Text>
              </CardBody>
            </Card>
          </SimpleGrid>
        </VStack>
      </Container>

      {/* Testimonials Section */}
      <Box bg="gray.50" py={20}>
        <Container maxW="7xl">
          <VStack spacing={16} w="full">
            <VStack spacing={6} textAlign="center">
              <Text
                fontSize={{ base: "3xl", md: "4xl", lg: "5xl" }}
                fontWeight="900"
                color="blue.900"
                lineHeight="1.2"
              >
                Hear it From Our{" "}
                <Text as="span" color="cyan.500">
                  Clients
                </Text>
              </Text>
              <Text
                fontSize="lg"
                color="gray.600"
                maxW="2xl"
                lineHeight="1.6"
              >
                Real feedback from businesses that have transformed with our AI solutions
              </Text>
            </VStack>
            
            {/* Single Testimonial */}
            <Card
              bg="white"
              border="2px solid"
              borderColor="blue.100"
              borderRadius="20px"
              boxShadow="0 20px 60px rgba(59, 130, 246, 0.1)"
              maxW="4xl"
              mx="auto"
              p={8}
            >
              <CardBody>
                <VStack spacing={8} textAlign="center">
                  {/* Quote Icon */}
                  <Icon as={FaQuoteLeft} fontSize="4xl" color="cyan.400" />
                  
                  {/* Testimonial Text */}
                  <Text
                    fontSize={{ base: "lg", md: "xl" }}
                    color="gray.700"
                    lineHeight="1.8"
                    fontStyle="italic"
                    maxW="3xl"
                  >
                    "The AI solutions provided by this team have completely revolutionized our manufacturing process. 
                    We've seen a 300% increase in productivity and a 50% reduction in operational costs. 
                    The implementation was seamless, and their ongoing support has been exceptional. 
                    I highly recommend their services to any business looking to leverage the power of artificial intelligence."
                  </Text>
                  
                  {/* Star Rating */}
                  <HStack spacing={1}>
                    <Icon as={FaStar} color="yellow.400" fontSize="xl" />
                    <Icon as={FaStar} color="yellow.400" fontSize="xl" />
                    <Icon as={FaStar} color="yellow.400" fontSize="xl" />
                    <Icon as={FaStar} color="yellow.400" fontSize="xl" />
                    <Icon as={FaStar} color="yellow.400" fontSize="xl" />
                  </HStack>
                  
                  {/* Client Info */}
                  <HStack spacing={4} align="center">
                    <Avatar
                      size="lg"
                      name="Sarah Johnson"
                      src="https://images.unsplash.com/photo-1494790108755-2616b612b7e8?ixlib=rb-4.0.3&auto=format&fit=crop&w=150&q=80"
                    />
                    <VStack align="start" spacing={0}>
                      <Text fontSize="lg" fontWeight="bold" color="blue.900">
                        Sarah Johnson
                      </Text>
                      <Text fontSize="md" color="gray.600">
                        CEO, TechCorp Industries
                      </Text>
                      <Text fontSize="sm" color="cyan.500">
                        Manufacturing & Automation
                      </Text>
                    </VStack>
                  </HStack>
                </VStack>
              </CardBody>
            </Card>
          </VStack>
        </Container>
      </Box>

      {/* FAQ + Contact Section */}
      <Container maxW="7xl" py={20}>
        <HStack
          spacing={20}
          align="start"
          w="full"
          flexDir={{ base: "column", lg: "row" }}
        >
          {/* Left Side - FAQ */}
          <VStack
            align="start"
            spacing={8}
            flex="1"
            w="full"
          >
            <VStack spacing={4} align="start" w="full">
              <Text
                fontSize={{ base: "2xl", md: "3xl", lg: "4xl" }}
                fontWeight="900"
                color="blue.900"
                lineHeight="1.2"
              >
                Frequently Asked{" "}
                <Text as="span" color="cyan.500">
                  Questions
                </Text>
              </Text>
              <Text
                fontSize="lg"
                color="gray.600"
                lineHeight="1.6"
              >
                Get answers to common questions about our AI and robotics solutions
              </Text>
            </VStack>
            
            <Card
              bg="white"
              border="2px solid"
              borderColor="blue.100"
              borderRadius="15px"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              w="full"
            >
              <CardBody p={4}>
                <Accordion allowMultiple>
                  <AccordionItem border="none">
                    <AccordionButton
                      _hover={{ bg: "blue.50" }}
                      borderRadius="md"
                      py={4}
                    >
                      <Box flex="1" textAlign="left">
                        <Text fontSize="lg" fontWeight="bold" color="blue.900">
                          What industries do you serve?
                        </Text>
                      </Box>
                      <AccordionIcon color="cyan.500" />
                    </AccordionButton>
                    <AccordionPanel pb={4} color="gray.600" fontSize="md" lineHeight="1.6">
                      We serve a wide range of industries including manufacturing, healthcare, 
                      retail, education, finance, and smart cities. Our AI solutions are customizable 
                      to meet the specific needs of any business sector.
                    </AccordionPanel>
                  </AccordionItem>

                  <AccordionItem border="none">
                    <AccordionButton
                      _hover={{ bg: "blue.50" }}
                      borderRadius="md"
                      py={4}
                    >
                      <Box flex="1" textAlign="left">
                        <Text fontSize="lg" fontWeight="bold" color="blue.900">
                          How long does implementation take?
                        </Text>
                      </Box>
                      <AccordionIcon color="cyan.500" />
                    </AccordionButton>
                    <AccordionPanel pb={4} color="gray.600" fontSize="md" lineHeight="1.6">
                      Implementation timelines vary based on project complexity. Simple automation 
                      projects can be deployed in 2-4 weeks, while comprehensive AI systems may 
                      take 3-6 months. We provide detailed timelines during consultation.
                    </AccordionPanel>
                  </AccordionItem>

                  <AccordionItem border="none">
                    <AccordionButton
                      _hover={{ bg: "blue.50" }}
                      borderRadius="md"
                      py={4}
                    >
                      <Box flex="1" textAlign="left">
                        <Text fontSize="lg" fontWeight="bold" color="blue.900">
                          Do you provide ongoing support?
                        </Text>
                      </Box>
                      <AccordionIcon color="cyan.500" />
                    </AccordionButton>
                    <AccordionPanel pb={4} color="gray.600" fontSize="md" lineHeight="1.6">
                      Yes, we offer comprehensive 24/7 support, regular system updates, 
                      performance monitoring, and continuous optimization to ensure your 
                      AI solutions deliver maximum value over time.
                    </AccordionPanel>
                  </AccordionItem>

                  <AccordionItem border="none">
                    <AccordionButton
                      _hover={{ bg: "blue.50" }}
                      borderRadius="md"
                      py={4}
                    >
                      <Box flex="1" textAlign="left">
                        <Text fontSize="lg" fontWeight="bold" color="blue.900">
                          What's the ROI of AI implementation?
                        </Text>
                      </Box>
                      <AccordionIcon color="cyan.500" />
                    </AccordionButton>
                    <AccordionPanel pb={4} color="gray.600" fontSize="md" lineHeight="1.6">
                      Our clients typically see ROI within 6-12 months, with average productivity 
                      increases of 200-400% and cost reductions of 30-60%. We provide detailed 
                      ROI projections during the initial consultation phase.
                    </AccordionPanel>
                  </AccordionItem>
                </Accordion>
              </CardBody>
            </Card>
          </VStack>
          
          {/* Right Side - Contact Form */}
          <VStack
            align="start"
            spacing={8}
            flex="1"
            w="full"
          >
            <VStack spacing={4} align="start" w="full">
              <Text
                fontSize={{ base: "2xl", md: "3xl", lg: "4xl" }}
                fontWeight="900"
                color="blue.900"
                lineHeight="1.2"
              >
                Get in{" "}
                <Text as="span" color="cyan.500">
                  Touch
                </Text>
              </Text>
              <Text
                fontSize="lg"
                color="gray.600"
                lineHeight="1.6"
              >
                Ready to transform your business with AI? Let's discuss your project.
              </Text>
            </VStack>
            
            <Card
              bg="white"
              border="2px solid"
              borderColor="blue.100"
              borderRadius="15px"
              boxShadow="0 10px 30px rgba(59, 130, 246, 0.1)"
              w="full"
            >
              <CardBody p={8}>
                <VStack spacing={6} w="full">
                  <FormControl>
                    <FormLabel color="blue.900" fontWeight="semibold">
                      Full Name
                    </FormLabel>
                    <Input
                      placeholder="Enter your full name"
                      size="lg"
                      borderColor="blue.200"
                      _hover={{ borderColor: "cyan.400" }}
                      _focus={{ borderColor: "cyan.500", boxShadow: "0 0 0 1px var(--chakra-colors-cyan-500)" }}
                    />
                  </FormControl>
                  
                  <FormControl>
                    <FormLabel color="blue.900" fontWeight="semibold">
                      Email Address
                    </FormLabel>
                    <Input
                      type="email"
                      placeholder="Enter your email address"
                      size="lg"
                      borderColor="blue.200"
                      _hover={{ borderColor: "cyan.400" }}
                      _focus={{ borderColor: "cyan.500", boxShadow: "0 0 0 1px var(--chakra-colors-cyan-500)" }}
                    />
                  </FormControl>
                  
                  <FormControl>
                    <FormLabel color="blue.900" fontWeight="semibold">
                      Message
                    </FormLabel>
                    <Textarea
                      placeholder="Tell us about your project and how we can help..."
                      rows={5}
                      resize="vertical"
                      borderColor="blue.200"
                      _hover={{ borderColor: "cyan.400" }}
                      _focus={{ borderColor: "cyan.500", boxShadow: "0 0 0 1px var(--chakra-colors-cyan-500)" }}
                    />
                  </FormControl>
                  
                  <Button
                    size="lg"
                    w="full"
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
                    leftIcon={<FaArrowRight />}
                  >
                    Send Message
                  </Button>
                </VStack>
              </CardBody>
            </Card>
          </VStack>
        </HStack>
      </Container>

      {/* Footer */}
      <Box
        position="relative"
        py={16}
        bgGradient="linear(135deg, blue.900 0%, blue.800 25%, blue.700 50%, blue.600 75%, blue.500 100%)"
        overflow="hidden"
      >
        {/* Background Tech Pattern */}
        <Box
          position="absolute"
          top="0"
          left="0"
          right="0"
          bottom="0"
          opacity="0.05"
          backgroundImage="url('data:image/svg+xml,<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 200 200\"><path d=\"M20 20h40v40h-40zM80 20h40v40h-40zM140 20h40v40h-40zM20 80h40v40h-40zM80 80h40v40h-40zM140 80h40v40h-40zM20 140h40v40h-40zM80 140h40v40h-40zM140 140h40v40h-40z\" fill=\"none\" stroke=\"%23ffffff\" stroke-width=\"2\"/><circle cx=\"40\" cy=\"40\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"100\" cy=\"40\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"160\" cy=\"40\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"40\" cy=\"100\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"100\" cy=\"100\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"160\" cy=\"100\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"40\" cy=\"160\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"100\" cy=\"160\" r=\"5\" fill=\"%23ffffff\"/><circle cx=\"160\" cy=\"160\" r=\"5\" fill=\"%23ffffff\"/></svg>')"
          backgroundSize="200px 200px"
          zIndex="0"
        />
        
        <Container maxW="7xl" position="relative" zIndex="1">
          <VStack spacing={12} w="full">
            <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={10} w="full">
              {/* Company Info */}
              <VStack align="start" spacing={6}>
                <HStack spacing={3}>
                  <Icon as={FaRobot} fontSize="3xl" color="cyan.300" />
                  <Text fontSize="2xl" fontWeight="bold" color="white">
                    AI Robotics
                  </Text>
                </HStack>
                <Text color="blue.100" lineHeight="1.6" maxW="sm">
                  Transforming businesses worldwide with cutting-edge artificial intelligence 
                  and robotics solutions. Building the future, one innovation at a time.
                </Text>
                <HStack spacing={4}>
                  <Link href="#" _hover={{ color: "cyan.300" }}>
                    <Icon as={FaFacebook} fontSize="xl" color="blue.200" />
                  </Link>
                  <Link href="#" _hover={{ color: "cyan.300" }}>
                    <Icon as={FaTwitter} fontSize="xl" color="blue.200" />
                  </Link>
                  <Link href="#" _hover={{ color: "cyan.300" }}>
                    <Icon as={FaLinkedin} fontSize="xl" color="blue.200" />
                  </Link>
                  <Link href="#" _hover={{ color: "cyan.300" }}>
                    <Icon as={FaGithub} fontSize="xl" color="blue.200" />
                  </Link>
                </HStack>
              </VStack>
              
              {/* Quick Links */}
              <VStack align="start" spacing={4}>
                <Text fontSize="lg" fontWeight="bold" color="cyan.300">
                  Quick Links
                </Text>
                <VStack align="start" spacing={2}>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    About Us
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Our Story
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Case Studies
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Blog
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Careers
                  </Link>
                </VStack>
              </VStack>
              
              {/* Services */}
              <VStack align="start" spacing={4}>
                <Text fontSize="lg" fontWeight="bold" color="cyan.300">
                  Services
                </Text>
                <VStack align="start" spacing={2}>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Robotic Automation
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Machine Learning
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Education & Science
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Predictive Analysis
                  </Link>
                  <Link href="#" color="blue.100" _hover={{ color: "white" }}>
                    Consulting
                  </Link>
                </VStack>
              </VStack>
              
              {/* Contact Info */}
              <VStack align="start" spacing={4}>
                <Text fontSize="lg" fontWeight="bold" color="cyan.300">
                  Contact Info
                </Text>
                <VStack align="start" spacing={3}>
                  <HStack spacing={3}>
                    <Icon as={FaMapMarkerAlt} color="cyan.400" />
                    <Text color="blue.100" fontSize="sm">
                      123 Innovation Drive, Tech City, TC 12345
                    </Text>
                  </HStack>
                  <HStack spacing={3}>
                    <Icon as={FaPhone} color="cyan.400" />
                    <Text color="blue.100" fontSize="sm">
                      +1 (555) 123-4567
                    </Text>
                  </HStack>
                  <HStack spacing={3}>
                    <Icon as={FaEnvelope} color="cyan.400" />
                    <Text color="blue.100" fontSize="sm">
                      contact@airobotics.com
                    </Text>
                  </HStack>
                </VStack>
              </VStack>
            </SimpleGrid>
            
            <Divider borderColor="blue.600" />
            
            {/* Copyright */}
            <Flex
              direction={{ base: "column", md: "row" }}
              justify="space-between"
              align="center"
              w="full"
              gap={4}
            >
              <Text color="blue.200" fontSize="sm">
                © 2024 AI Robotics. All rights reserved.
              </Text>
              <HStack spacing={6}>
                <Link href="#" color="blue.200" fontSize="sm" _hover={{ color: "white" }}>
                  Privacy Policy
                </Link>
                <Link href="#" color="blue.200" fontSize="sm" _hover={{ color: "white" }}>
                  Terms of Service
                </Link>
                <Link href="#" color="blue.200" fontSize="sm" _hover={{ color: "white" }}>
                  Cookie Policy
                </Link>
              </HStack>
            </Flex>
          </VStack>
        </Container>
      </Box>
    </Box>
  );
};

export default LandingPage;