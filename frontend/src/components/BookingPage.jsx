import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Container,
  Card,
  CardBody,
  CardHeader,
  SimpleGrid,
  Badge,
  Avatar,
  Divider,
  useToast,
  Select,
  Input,
  FormControl,
  FormLabel,
} from "@chakra-ui/react";
import { useState, useEffect } from "react";

// Dummy data for available time slots
const generateTimeSlots = () => {
  const slots = [];
  const today = new Date();
  
  for (let day = 0; day < 7; day++) {
    const date = new Date(today);
    date.setDate(today.getDate() + day);
    
    // Generate slots from 9 AM to 9 PM
    for (let hour = 9; hour <= 21; hour++) {
      const startTime = new Date(date);
      startTime.setHours(hour, 0, 0, 0);
      
      const endTime = new Date(startTime);
      endTime.setHours(hour + 1, 0, 0, 0);
      
      // Randomly mark some slots as taken to simulate real usage
      const isTaken = Math.random() < 0.3;
      
      slots.push({
        id: `slot_${day}_${hour}`,
        date: date.toISOString().split('T')[0],
        startTime: startTime.toLocaleTimeString('en-US', { 
          hour: 'numeric', 
          minute: '2-digit',
          hour12: true 
        }),
        endTime: endTime.toLocaleTimeString('en-US', { 
          hour: 'numeric', 
          minute: '2-digit',
          hour12: true 
        }),
        available: !isTaken,
        robotType: ["turtlebot", "arm", "hand"][Math.floor(Math.random() * 3)],
        bookedBy: isTaken ? "Another User" : null,
      });
    }
  }
  
  return slots;
};

const robotNames = {
  turtlebot: { name: "TurtleBot3", emoji: "🤖" },
  arm: { name: "Robot Arm", emoji: "🦾" },
  hand: { name: "Robot Hand", emoji: "🤲" },
};

const BookingPage = ({ user, onBooking, onLogout }) => {
  const [timeSlots, setTimeSlots] = useState([]);
  const [selectedDate, setSelectedDate] = useState("");
  const [selectedRobot, setSelectedRobot] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const toast = useToast();

  useEffect(() => {
    // Generate dummy time slots on component mount
    const slots = generateTimeSlots();
    setTimeSlots(slots);
    
    // Set default date to today
    const today = new Date().toISOString().split('T')[0];
    setSelectedDate(today);
  }, []);

  const filteredSlots = timeSlots.filter(slot => {
    let matches = true;
    if (selectedDate && slot.date !== selectedDate) matches = false;
    if (selectedRobot && slot.robotType !== selectedRobot) matches = false;
    return matches;
  });

  const availableSlots = filteredSlots.filter(slot => slot.available);
  const bookedSlots = filteredSlots.filter(slot => !slot.available);

  const handleBookSlot = async (slot) => {
    setIsLoading(true);
    
    // Simulate booking API call
    setTimeout(() => {
      const bookedSlot = {
        ...slot,
        available: false,
        bookedBy: user.name,
        bookingTime: new Date().toISOString(),
      };
      
      // Update the slot status
      setTimeSlots(prev => prev.map(s => 
        s.id === slot.id ? bookedSlot : s
      ));
      
      onBooking(bookedSlot);
      
      toast({
        title: "Booking confirmed!",
        description: `Your session is booked for ${slot.date} at ${slot.startTime}`,
        status: "success",
        duration: 5000,
        isClosable: true,
      });
      
      setIsLoading(false);
    }, 1000);
  };

  const getDateOptions = () => {
    const today = new Date();
    const options = [];
    
    for (let i = 0; i < 7; i++) {
      const date = new Date(today);
      date.setDate(today.getDate() + i);
      const dateStr = date.toISOString().split('T')[0];
      const label = i === 0 ? 'Today' : 
                   i === 1 ? 'Tomorrow' : 
                   date.toLocaleDateString('en-US', { weekday: 'long', month: 'short', day: 'numeric' });
      options.push({ value: dateStr, label });
    }
    
    return options;
  };

  return (
    <Box minH="100vh" position="relative">
      {/* Enhanced background */}
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
      
      {/* Subtle background animation */}
      <Box
        position="absolute"
        top={0}
        left={0}
        right={0}
        bottom={0}
        overflow="hidden"
        zIndex={-1}
      >
        {[...Array(8)].map((_, i) => (
          <Box
            key={i}
            position="absolute"
            w="2px"
            h="2px"
            bg="robotics.accent"
            borderRadius="full"
            opacity={0.1}
            animation={`float ${5 + i * 0.4}s ease-in-out infinite alternate`}
            style={{
              left: `${Math.random() * 100}%`,
              top: `${Math.random() * 100}%`,
              animationDelay: `${i * 0.4}s`,
            }}
          />
        ))}
      </Box>

    <Container maxW="7xl" py={8} position="relative" zIndex={1}>
      <VStack spacing={10}>
        {/* Enhanced Header */}
        <Card variant="gradient" w="full" p={6}>
          <HStack w="full" justify="space-between">
            <VStack align="start" spacing={3}>
              <Text 
                fontSize={{ base: "2xl", md: "3xl" }} 
                fontWeight="bold" 
                bgGradient="linear(to-r, white, robotics.primary)"
                bgClip="text"
              >
                Book Your Robot Session
              </Text>
              <HStack spacing={3}>
                <Avatar 
                  size="sm" 
                  name={user.name} 
                  border="2px solid"
                  borderColor="robotics.primary"
                />
                <Text color="gray.200" fontSize="lg">Welcome, {user.name}</Text>
              </HStack>
            </VStack>
            <Button 
              variant="ghost" 
              onClick={onLogout} 
              color="gray.400"
              size="lg"
              _hover={{
                color: "robotics.secondary",
                bg: "whiteAlpha.100"
              }}
            >
              Logout
            </Button>
          </HStack>
        </Card>

        {/* Enhanced Filters */}
        <Card variant="gradient" w="full" border="1px solid" borderColor="dark.border">
          <CardHeader>
            <HStack>
              <Text fontSize="xl" fontWeight="bold" color="white">
                Filter Available Sessions
              </Text>
              <Badge variant="glow">
                {availableSlots.length} Available
              </Badge>
            </HStack>
          </CardHeader>
          <CardBody>
            <SimpleGrid columns={{ base: 1, md: 2 }} spacing={6}>
              <FormControl>
                <FormLabel 
                  color="gray.300" 
                  fontSize="md" 
                  fontWeight="semibold"
                  mb={3}
                >
                  Date
                </FormLabel>
                <Select
                  value={selectedDate}
                  onChange={(e) => setSelectedDate(e.target.value)}
                  size="lg"
                  borderRadius="lg"
                  _focus={{
                    borderColor: "robotics.primary",
                    boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                  }}
                >
                  <option value="">All dates</option>
                  {getDateOptions().map(option => (
                    <option key={option.value} value={option.value}>
                      {option.label}
                    </option>
                  ))}
                </Select>
              </FormControl>

              <FormControl>
                <FormLabel 
                  color="gray.300" 
                  fontSize="md" 
                  fontWeight="semibold"
                  mb={3}
                >
                  Robot Type
                </FormLabel>
                <Select
                  value={selectedRobot}
                  onChange={(e) => setSelectedRobot(e.target.value)}
                  size="lg"
                  borderRadius="lg"
                  _focus={{
                    borderColor: "robotics.primary",
                    boxShadow: "0 0 0 3px rgba(0,212,255,0.1)"
                  }}
                >
                  <option value="">All robots</option>
                  <option value="turtlebot">🤖 TurtleBot3</option>
                  <option value="arm">🦾 Robot Arm</option>
                  <option value="hand">🤲 Robot Hand</option>
                </Select>
              </FormControl>
            </SimpleGrid>
          </CardBody>
        </Card>
        {/* Enhanced Available Slots */}
        <VStack w="full" spacing={8}>
          <HStack w="full" justify="space-between" align="center">
            <Text 
              fontSize={{ base: "xl", md: "2xl" }} 
              fontWeight="bold" 
              bgGradient="linear(to-r, white, robotics.accent)"
              bgClip="text"
            >
              Available Sessions ({availableSlots.length})
            </Text>
            <Badge variant="robotics" px={4} py={2} fontSize="sm" borderRadius="full">
              {availableSlots.length} slots free
            </Badge>
          </HStack>

          {availableSlots.length === 0 ? (
            <Card variant="gradient" w="full" border="1px solid" borderColor="dark.border">
              <CardBody textAlign="center" py={16}>
                <Text fontSize="6xl" mb={4}>😔</Text>
                <Text fontSize="xl" color="gray.300" mb={3}>
                  No available slots found
                </Text>
                <Text color="gray.400">
                  Try adjusting your filters or check back later
                </Text>
              </CardBody>
            </Card>
          ) : (
            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={6} w="full">
              {availableSlots.map((slot) => (
                <Card
                  key={slot.id}
                  variant="gradient"
                  cursor="pointer"
                  position="relative"
                  overflow="hidden"
                  _hover={{
                    transform: "translateY(-4px) scale(1.02)",
                    boxShadow: "0 10px 30px rgba(0,212,255,0.2)",
                  }}
                  transition="all 0.3s ease-in-out"
                >
                  {/* Hover effect */}
                  <Box
                    position="absolute"
                    top={0}
                    left={0}
                    right={0}
                    bottom={0}
                    bgGradient="linear(to-br, rgba(0,212,255,0.1), rgba(78,205,196,0.1))"
                    opacity={0}
                    transition="opacity 0.3s"
                    _groupHover={{ opacity: 1 }}
                  />
                  
                  <CardBody p={6} position="relative" zIndex={1}>
                    <VStack spacing={4} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge variant="robotics" size="sm">Available</Badge>
                        <HStack spacing={2}>
                          <Text fontSize="2xl">{robotNames[slot.robotType].emoji}</Text>
                          <Text fontSize="sm" color="gray.300" fontWeight="semibold">
                            {robotNames[slot.robotType].name}
                          </Text>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={2} w="full">
                        <Text color="white" fontWeight="bold" fontSize="lg">
                          {new Date(slot.date).toLocaleDateString('en-US', { 
                            weekday: 'long', 
                            month: 'long', 
                            day: 'numeric' 
                          })}
                        </Text>
                        <Text color="robotics.primary" fontSize="xl" fontWeight="bold">
                          {slot.startTime} - {slot.endTime}
                        </Text>
                      </VStack>

                      <Button
                        variant="robotics"
                        w="full"
                        size="lg"
                        onClick={() => handleBookSlot(slot)}
                        isLoading={isLoading}
                        loadingText="Booking..."
                        leftIcon={<Text fontSize="lg">🚀</Text>}
                        mt={2}
                      >
                        Book This Session
                      </Button>
                    </VStack>
                  </CardBody>
                </Card>
              ))}
            </SimpleGrid>
          )}
        </VStack>

        {/* Enhanced Booked Slots (for reference) */}
        {bookedSlots.length > 0 && (
          <VStack w="full" spacing={8}>
            <Divider borderColor="dark.border" />
            
            <HStack w="full" justify="space-between" align="center">
              <Text 
                fontSize={{ base: "xl", md: "2xl" }} 
                fontWeight="bold" 
                bgGradient="linear(to-r, gray.400, robotics.secondary)"
                bgClip="text"
              >
                Unavailable Sessions
              </Text>
              <Badge variant="outline" colorScheme="red" px={4} py={2} fontSize="sm" borderRadius="full">
                {bookedSlots.length} slots taken
              </Badge>
            </HStack>

            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={6} w="full">
              {bookedSlots.slice(0, 6).map((slot) => (
                <Card
                  key={slot.id}
                  bg="whiteAlpha.50"
                  border="1px solid"
                  borderColor="gray.700"
                  opacity={0.6}
                  position="relative"
                >
                  <CardBody p={6}>
                    <VStack spacing={4} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge colorScheme="red" variant="subtle">Taken</Badge>
                        <HStack spacing={2}>
                          <Text fontSize="2xl" opacity={0.6}>{robotNames[slot.robotType].emoji}</Text>
                          <Text fontSize="sm" color="gray.400" fontWeight="semibold">
                            {robotNames[slot.robotType].name}
                          </Text>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={2} w="full">
                        <Text color="gray.300" fontWeight="bold">
                          {new Date(slot.date).toLocaleDateString('en-US', { 
                            weekday: 'long', 
                            month: 'long', 
                            day: 'numeric' 
                          })}
                        </Text>
                        <Text color="gray.400" fontSize="lg">
                          {slot.startTime} - {slot.endTime}
                        </Text>
                      </VStack>

                      <Box
                        w="full"
                        p={3}
                        bg="red.900"
                        borderRadius="md"
                        border="1px solid"
                        borderColor="red.700"
                      >
                        <Text color="red.200" fontSize="sm" textAlign="center" fontWeight="semibold">
                          Booked by {slot.bookedBy}
                        </Text>
                      </Box>
                    </VStack>
                  </CardBody>
                </Card>
              ))}
            </SimpleGrid>
          </VStack>
        )}
      </VStack>
      </Container>
    </Box>
  );
};

export default BookingPage;
