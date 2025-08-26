import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Input,
  FormControl,
  FormLabel,
  Container,
  Card,
  CardBody,
  CardHeader,
  Tabs,
  TabList,
  TabPanels,
  Tab,
  TabPanel,
  useToast,
  Alert,
  AlertIcon,
  InputGroup,
  InputLeftElement,
  Icon,
  keyframes,
} from "@chakra-ui/react";
import { useState } from "react";
import { motion } from "framer-motion";
import { FiMail, FiLock, FiUser } from "react-icons/fi";

const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);

// Animation keyframes
const float = keyframes`
  0% { transform: translateY(0px); }
  50% { transform: translateY(-5px); }
  100% { transform: translateY(0px); }
`;

const fadeInUp = {
  hidden: { opacity: 0, y: 30 },
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

const AuthPage = ({ onAuth, onBack }) => {
  const [isLoading, setIsLoading] = useState(false);
  const [loginData, setLoginData] = useState({ email: "", password: "" });
  const [registerData, setRegisterData] = useState({
    name: "",
    email: "",
    password: "",
    confirmPassword: ""
  });
  const toast = useToast();

  const handleLogin = async (e) => {
    e.preventDefault();
    setIsLoading(true);

    // Simulate API call with dummy validation
    setTimeout(() => {
      if (loginData.email && loginData.password) {
        const userData = {
          id: "user_123",
          name: loginData.email.split("@")[0],
          email: loginData.email,
        };
        onAuth(userData);
        toast({
          title: "Login successful",
          status: "success",
          duration: 3000,
          isClosable: true,
        });
      } else {
        toast({
          title: "Login failed",
          description: "Please enter both email and password",
          status: "error",
          duration: 3000,
          isClosable: true,
        });
      }
      setIsLoading(false);
    }, 1000);
  };

  const handleRegister = async (e) => {
    e.preventDefault();
    setIsLoading(true);

    // Simulate API call with dummy validation
    setTimeout(() => {
      if (registerData.name && registerData.email && registerData.password) {
        if (registerData.password !== registerData.confirmPassword) {
          toast({
            title: "Registration failed",
            description: "Passwords do not match",
            status: "error",
            duration: 3000,
            isClosable: true,
          });
          setIsLoading(false);
          return;
        }

        const userData = {
          id: "user_" + Date.now(),
          name: registerData.name,
          email: registerData.email,
        };
        onAuth(userData);
        toast({
          title: "Registration successful",
          description: "Welcome to Robot Programming Console!",
          status: "success",
          duration: 3000,
          isClosable: true,
        });
      } else {
        toast({
          title: "Registration failed",
          description: "Please fill in all fields",
          status: "error",
          duration: 3000,
          isClosable: true,
        });
      }
      setIsLoading(false);
    }, 1000);
  };

  return (
    <Container maxW="md" py={20}>
      <MotionVStack 
        spacing={10}
        initial="hidden"
        animate="visible"
        variants={staggerContainer}
      >
        {/* Enhanced Header */}
        <MotionVStack spacing={6} textAlign="center" variants={fadeInUp}>
          <MotionBox
            fontSize="6xl"
            animation={`${float} 3s ease-in-out infinite`}
          >
            🤖
          </MotionBox>
          <VStack spacing={3}>
            <Text 
              fontSize={{ base: "2xl", md: "3xl" }} 
              fontWeight="bold" 
              bgGradient="linear(to-r, #667eea, #764ba2)"
              bgClip="text"
            >
              Welcome Back
            </Text>
            <Text color="gray.300" fontSize="lg" lineHeight="relaxed">
              Sign in to book your robot programming session
            </Text>
          </VStack>
        </MotionVStack>

        {/* Enhanced Form Card */}
        <MotionCard 
          w="full" 
          variant="floating"
          variants={fadeInUp}
        >
          <CardBody p={8}>
            <Tabs variant="enclosed" colorScheme="blue">
              <TabList 
                border="none" 
                bg="rgba(255, 255, 255, 0.05)"
                borderRadius="lg"
                p={1}
              >
                <Tab 
                  color="gray.300" 
                  borderRadius="md"
                  _selected={{ 
                    color: "white", 
                    bg: "linear-gradient(135deg, #667eea 0%, #764ba2 100%)",
                    transform: "scale(1.02)"
                  }}
                  transition="all 0.3s ease"
                >
                  Sign In
                </Tab>
                <Tab 
                  color="gray.300" 
                  borderRadius="md"
                  _selected={{ 
                    color: "white", 
                    bg: "linear-gradient(135deg, #667eea 0%, #764ba2 100%)",
                    transform: "scale(1.02)"
                  }}
                  transition="all 0.3s ease"
                >
                  Sign Up
                </Tab>
              </TabList>

              <TabPanels>
                {/* Enhanced Login Tab */}
                <TabPanel px={0} pt={8}>
                  <form onSubmit={handleLogin}>
                    <VStack spacing={6}>
                      <FormControl>
                        <InputGroup>
                          <InputLeftElement>
                            <Icon as={FiMail} color="gray.400" />
                          </InputLeftElement>
                          <Input
                            type="email"
                            placeholder="Enter your email"
                            value={loginData.email}
                            onChange={(e) => setLoginData({...loginData, email: e.target.value})}
                            variant="glass"
                            size="lg"
                            pl={12}
                          />
                        </InputGroup>
                      </FormControl>

                      <FormControl>
                        <InputGroup>
                          <InputLeftElement>
                            <Icon as={FiLock} color="gray.400" />
                          </InputLeftElement>
                          <Input
                            type="password"
                            placeholder="Enter your password"
                            value={loginData.password}
                            onChange={(e) => setLoginData({...loginData, password: e.target.value})}
                            variant="glass"
                            size="lg"
                            pl={12}
                          />
                        </InputGroup>
                      </FormControl>

                      <Button
                        type="submit"
                        variant="gradient"
                        w="full"
                        size="lg"
                        isLoading={isLoading}
                        loadingText="Signing In..."
                        borderRadius="xl"
                      >
                        Sign In
                      </Button>
                    </VStack>
                  </form>
                </TabPanel>

                {/* Enhanced Register Tab */}
                <TabPanel px={0} pt={8}>
                  <form onSubmit={handleRegister}>
                    <VStack spacing={6}>
                      <FormControl>
                        <InputGroup>
                          <InputLeftElement>
                            <Icon as={FiUser} color="gray.400" />
                          </InputLeftElement>
                          <Input
                            type="text"
                            placeholder="Enter your full name"
                            value={registerData.name}
                            onChange={(e) => setRegisterData({...registerData, name: e.target.value})}
                            variant="glass"
                            size="lg"
                            pl={12}
                          />
                        </InputGroup>
                      </FormControl>

                      <FormControl>
                        <InputGroup>
                          <InputLeftElement>
                            <Icon as={FiMail} color="gray.400" />
                          </InputLeftElement>
                          <Input
                            type="email"
                            placeholder="Enter your email"
                            value={registerData.email}
                            onChange={(e) => setRegisterData({...registerData, email: e.target.value})}
                            variant="glass"
                            size="lg"
                            pl={12}
                          />
                        </InputGroup>
                      </FormControl>

                      <FormControl>
                        <InputGroup>
                          <InputLeftElement>
                            <Icon as={FiLock} color="gray.400" />
                          </InputLeftElement>
                          <Input
                            type="password"
                            placeholder="Create password"
                            value={registerData.password}
                            onChange={(e) => setRegisterData({...registerData, password: e.target.value})}
                            variant="glass"
                            size="lg"
                            pl={12}
                          />
                        </InputGroup>
                      </FormControl>

                      <FormControl>
                        <InputGroup>
                          <InputLeftElement>
                            <Icon as={FiLock} color="gray.400" />
                          </InputLeftElement>
                          <Input
                            type="password"
                            placeholder="Confirm password"
                            value={registerData.confirmPassword}
                            onChange={(e) => setRegisterData({...registerData, confirmPassword: e.target.value})}
                            variant="glass"
                            size="lg"
                            pl={12}
                          />
                        </InputGroup>
                      </FormControl>

                      <Button
                        type="submit"
                        variant="success"
                        w="full"
                        size="lg"
                        isLoading={isLoading}
                        loadingText="Creating Account..."
                        borderRadius="xl"
                      >
                        Create Account
                      </Button>
                    </VStack>
                  </form>
                </TabPanel>
              </TabPanels>
            </Tabs>
          </CardBody>
        </MotionCard>

        {/* Enhanced Demo Alert */}
        <MotionBox variants={fadeInUp} w="full">
          <Alert 
            status="info" 
            variant="glass"
            borderRadius="xl"
            backdropFilter="blur(10px)"
            bg="rgba(79, 172, 254, 0.1)"
            border="1px solid rgba(79, 172, 254, 0.2)"
          >
            <AlertIcon color="blue.300" />
            <Text color="gray.200">
              This is a demo system. Use any email/password to test the functionality.
            </Text>
          </Alert>
        </MotionBox>

        {/* Enhanced Back Button */}
        <MotionBox variants={fadeInUp}>
          <Button 
            variant="glass" 
            onClick={onBack}
            size="lg"
            borderRadius="xl"
          >
            ← Back to Home
          </Button>
        </MotionBox>
      </MotionVStack>
    </Container>
  );
};

export default AuthPage;