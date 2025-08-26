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
} from "@chakra-ui/react";
import { useState } from "react";

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
    <Box minH="100vh" position="relative">
      {/* Enhanced background with subtle animation */}
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
      
      {/* Animated background elements */}
      <Box
        position="absolute"
        top={0}
        left={0}
        right={0}
        bottom={0}
        overflow="hidden"
        zIndex={-1}
      >
        {[...Array(10)].map((_, i) => (
          <Box
            key={i}
            position="absolute"
            w="3px"
            h="3px"
            bg="robotics.primary"
            borderRadius="full"
            opacity={0.2}
            animation={`float ${4 + i * 0.3}s ease-in-out infinite alternate`}
            style={{
              left: `${Math.random() * 100}%`,
              top: `${Math.random() * 100}%`,
              animationDelay: `${i * 0.3}s`,
            }}
          />
        ))}
      </Box>

      <Container maxW="md" py={20} position="relative" zIndex={1}>
        <VStack spacing={10}>
          {/* Enhanced header section */}
          <VStack spacing={6} textAlign="center">
            <Box position="relative">
              <Text fontSize="6xl" mb={4}>🤖</Text>
              <Box
                position="absolute"
                top="50%"
                left="50%"
                transform="translate(-50%, -50%)"
                w="120px"
                h="120px"
                bgGradient="radial(circle, robotics.primary 0%, transparent 70%)"
                opacity={0.2}
                borderRadius="full"
                animation="pulse 3s ease-in-out infinite"
              />
            </Box>
            
            <VStack spacing={3}>
              <Text 
                fontSize={{ base: "2xl", md: "3xl" }} 
                fontWeight="bold" 
                bgGradient="linear(to-r, white, robotics.primary)"
                bgClip="text"
              >
                Welcome Back
              </Text>
              <Text 
                color="gray.300" 
                fontSize="lg"
                maxW="sm"
                lineHeight="tall"
              >
                Sign in to book your robot programming session and start your journey
              </Text>
            </VStack>
          </VStack>

          {/* Enhanced form card */}
          <Card 
            w="full" 
            variant="gradient" 
            border="2px solid"
            borderColor="dark.border"
            boxShadow="0 10px 40px rgba(0, 0, 0, 0.3)"
          >
            <CardBody p={8}>
              <Tabs variant="enclosed" colorScheme="brand">
                <TabList borderColor="dark.border" mb={6}>
                  <Tab 
                    color="gray.400" 
                    fontSize="lg"
                    fontWeight="semibold"
                    _selected={{ 
                      color: "white", 
                      bg: "robotics.primary",
                      borderColor: "robotics.primary",
                      boxShadow: "0 0 20px rgba(0,212,255,0.3)"
                    }}
                    _hover={{
                      color: "robotics.primary"
                    }}
                    transition="all 0.2s"
                  >
                    Sign In
                  </Tab>
                  <Tab 
                    color="gray.400" 
                    fontSize="lg"
                    fontWeight="semibold"
                    _selected={{ 
                      color: "white", 
                      bg: "robotics.primary",
                      borderColor: "robotics.primary",
                      boxShadow: "0 0 20px rgba(0,212,255,0.3)"
                    }}
                    _hover={{
                      color: "robotics.primary"
                    }}
                    transition="all 0.2s"
                  >
                    Sign Up
                  </Tab>
                </TabList>

                <TabPanels>
                  {/* Enhanced Login Tab */}
                  <TabPanel px={0}>
                    <form onSubmit={handleLogin}>
                      <VStack spacing={6}>
                        <FormControl>
                          <FormLabel 
                            color="gray.300" 
                            fontSize="md" 
                            fontWeight="semibold"
                            mb={3}
                          >
                            Email Address
                          </FormLabel>
                          <Input
                            type="email"
                            placeholder="Enter your email"
                            value={loginData.email}
                            onChange={(e) => setLoginData({...loginData, email: e.target.value})}
                            size="lg"
                            borderRadius="lg"
                            _focus={{
                              borderColor: "robotics.primary",
                              boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                            }}
                          />
                        </FormControl>

                        <FormControl>
                          <FormLabel 
                            color="gray.300" 
                            fontSize="md" 
                            fontWeight="semibold"
                            mb={3}
                          >
                            Password
                          </FormLabel>
                          <Input
                            type="password"
                            placeholder="Enter your password"
                            value={loginData.password}
                            onChange={(e) => setLoginData({...loginData, password: e.target.value})}
                            size="lg"
                            borderRadius="lg"
                            _focus={{
                              borderColor: "robotics.primary",
                              boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                            }}
                          />
                        </FormControl>

                        <Button
                          type="submit"
                          variant="robotics"
                          size="lg"
                          w="full"
                          py={6}
                          h="auto"
                          fontSize="lg"
                          isLoading={isLoading}
                          loadingText="Signing in..."
                          leftIcon={<Text fontSize="xl">🔐</Text>}
                        >
                          Sign In
                        </Button>
                      </VStack>
                    </form>
                  </TabPanel>

                {/* Enhanced Register Tab */}
                <TabPanel px={0}>
                  <form onSubmit={handleRegister}>
                    <VStack spacing={6}>
                      <FormControl>
                        <FormLabel 
                          color="gray.300" 
                          fontSize="md" 
                          fontWeight="semibold"
                          mb={3}
                        >
                          Full Name
                        </FormLabel>
                        <Input
                          type="text"
                          placeholder="Enter your full name"
                          value={registerData.name}
                          onChange={(e) => setRegisterData({...registerData, name: e.target.value})}
                          size="lg"
                          borderRadius="lg"
                          _focus={{
                            borderColor: "robotics.primary",
                            boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                          }}
                        />
                      </FormControl>

                      <FormControl>
                        <FormLabel 
                          color="gray.300" 
                          fontSize="md" 
                          fontWeight="semibold"
                          mb={3}
                        >
                          Email Address
                        </FormLabel>
                        <Input
                          type="email"
                          placeholder="Enter your email"
                          value={registerData.email}
                          onChange={(e) => setRegisterData({...registerData, email: e.target.value})}
                          size="lg"
                          borderRadius="lg"
                          _focus={{
                            borderColor: "robotics.primary",
                            boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                          }}
                        />
                      </FormControl>

                      <FormControl>
                        <FormLabel 
                          color="gray.300" 
                          fontSize="md" 
                          fontWeight="semibold"
                          mb={3}
                        >
                          Password
                        </FormLabel>
                        <Input
                          type="password"
                          placeholder="Choose a password"
                          value={registerData.password}
                          onChange={(e) => setRegisterData({...registerData, password: e.target.value})}
                          size="lg"
                          borderRadius="lg"
                          _focus={{
                            borderColor: "robotics.primary",
                            boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                          }}
                        />
                      </FormControl>

                      <FormControl>
                        <FormLabel 
                          color="gray.300" 
                          fontSize="md" 
                          fontWeight="semibold"
                          mb={3}
                        >
                          Confirm Password
                        </FormLabel>
                        <Input
                          type="password"
                          placeholder="Confirm your password"
                          value={registerData.confirmPassword}
                          onChange={(e) => setRegisterData({...registerData, confirmPassword: e.target.value})}
                          size="lg"
                          borderRadius="lg"
                          _focus={{
                            borderColor: "robotics.primary",
                            boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                          }}
                        />
                      </FormControl>

                      <Button
                        type="submit"
                        variant="gradient"
                        size="lg"
                        w="full"
                        py={6}
                        h="auto"
                        fontSize="lg"
                        isLoading={isLoading}
                        loadingText="Creating account..."
                        leftIcon={<Text fontSize="xl">✨</Text>}
                      >
                        Create Account
                      </Button>
                    </VStack>
                  </form>
                </TabPanel>
              </TabPanels>
            </Tabs>
          </CardBody>
        </Card>

        {/* Enhanced info alert */}
        <Alert 
          status="info" 
          bg="rgba(0,212,255,0.1)" 
          color="robotics.primary" 
          border="1px solid" 
          borderColor="robotics.primary"
          borderRadius="lg"
          boxShadow="0 0 20px rgba(0,212,255,0.1)"
        >
          <AlertIcon color="robotics.primary" />
          <Text fontSize="sm" lineHeight="tall">
            This is a demo system. Use any email/password to test the functionality and explore the robotics platform.
          </Text>
        </Alert>

        {/* Enhanced back button */}
        <Button 
          variant="ghost" 
          onClick={onBack} 
          color="gray.400"
          size="lg"
          leftIcon={<Text fontSize="lg">←</Text>}
          _hover={{
            color: "robotics.primary",
            bg: "whiteAlpha.100",
            transform: "translateX(-4px)"
          }}
          transition="all 0.2s"
        >
          Back to Home
        </Button>
      </VStack>
      </Container>
    </Box>
  );
};

export default AuthPage;