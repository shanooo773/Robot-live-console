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
  turtlebot: { name: "TurtleBot3 Navigation", emoji: "🤖" },
  arm: { name: "Robot Arm Manipulation", emoji: "🦾" },
  hand: { name: "Dexterous Hand Control", emoji: "🤲" },
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
        title: "Development session booked!",
        description: `Your coding console is reserved for ${slot.date} at ${slot.startTime}`,
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
    <Container maxW="7xl" py={8}>
      <VStack spacing={8}>
        {/* Header */}
        <HStack w="full" justify="space-between">
          <VStack align="start" spacing={1}>
            <Text fontSize="3xl" fontWeight="bold" color="white">
              Book Development Console Session
            </Text>
            <HStack>
              <Avatar size="sm" name={user.name} />
              <Text color="gray.300">Welcome, {user.name}</Text>
            </HStack>
          </VStack>
          <Button variant="ghost" onClick={onLogout} color="gray.400">
            Logout
          </Button>
        </HStack>

        {/* Filters */}
        <Card w="full" bg="gray.800" border="1px solid" borderColor="gray.600">
          <CardHeader>
            <Text fontSize="lg" fontWeight="bold" color="white">
              Choose Development Environment
            </Text>
          </CardHeader>
          <CardBody>
            <SimpleGrid columns={{ base: 1, md: 2 }} spacing={4}>
              <FormControl>
                <FormLabel color="gray.300">Date</FormLabel>
                <Select
                  value={selectedDate}
                  onChange={(e) => setSelectedDate(e.target.value)}
                  bg="gray.700"
                  border="1px solid"
                  borderColor="gray.600"
                  color="white"
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
                <FormLabel color="gray.300">Development Environment</FormLabel>
                <Select
                  value={selectedRobot}
                  onChange={(e) => setSelectedRobot(e.target.value)}
                  bg="gray.700"
                  border="1px solid"
                  borderColor="gray.600"
                  color="white"
                >
                  <option value="">All environments</option>
                  <option value="turtlebot">🤖 TurtleBot3 Navigation</option>
                  <option value="arm">🦾 Robot Arm Manipulation</option>
                  <option value="hand">🤲 Dexterous Hand Control</option>
                </Select>
              </FormControl>
            </SimpleGrid>
          </CardBody>
        </Card>

        {/* Available Slots */}
        <VStack w="full" spacing={6}>
          <HStack w="full" justify="space-between">
            <Text fontSize="xl" fontWeight="bold" color="white">
              Available Development Sessions ({availableSlots.length})
            </Text>
            <Badge colorScheme="green" px={3} py={1}>
              {availableSlots.length} console slots free
            </Badge>
          </HStack>

          {availableSlots.length === 0 ? (
            <Card w="full" bg="gray.800" border="1px solid" borderColor="gray.600">
              <CardBody textAlign="center" py={12}>
                <Text fontSize="xl" color="gray.400" mb={2}>
                  No development sessions available
                </Text>
                <Text color="gray.500">
                  Try adjusting your filters or check back later
                </Text>
              </CardBody>
            </Card>
          ) : (
            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={4} w="full">
              {availableSlots.map((slot) => (
                <Card
                  key={slot.id}
                  bg="gray.800"
                  border="1px solid"
                  borderColor="gray.600"
                  _hover={{ borderColor: "green.400", transform: "translateY(-2px)" }}
                  transition="all 0.2s"
                >
                  <CardBody>
                    <VStack spacing={3} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge colorScheme="green">Available</Badge>
                        <HStack>
                          <Text fontSize="xl">{robotNames[slot.robotType].emoji}</Text>
                          <Text fontSize="sm" color="gray.300">
                            {robotNames[slot.robotType].name}
                          </Text>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={1}>
                        <Text color="white" fontWeight="bold">
                          {new Date(slot.date).toLocaleDateString('en-US', { 
                            weekday: 'long', 
                            month: 'long', 
                            day: 'numeric' 
                          })}
                        </Text>
                        <Text color="gray.300" fontSize="lg">
                          {slot.startTime} - {slot.endTime}
                        </Text>
                      </VStack>

                      <Button
                        colorScheme="green"
                        w="full"
                        onClick={() => handleBookSlot(slot)}
                        isLoading={isLoading}
                        loadingText="Booking..."
                      >
                        Access Development Console
                      </Button>
                    </VStack>
                  </CardBody>
                </Card>
              ))}
            </SimpleGrid>
          )}
        </VStack>

        {/* Booked Slots (for reference) */}
        {bookedSlots.length > 0 && (
          <VStack w="full" spacing={6}>
            <Divider borderColor="gray.600" />
            
            <HStack w="full" justify="space-between">
              <Text fontSize="xl" fontWeight="bold" color="white">
                Unavailable Sessions
              </Text>
              <Badge colorScheme="red" px={3} py={1}>
                {bookedSlots.length} slots taken
              </Badge>
            </HStack>

            <SimpleGrid columns={{ base: 1, md: 2, lg: 3 }} spacing={4} w="full">
              {bookedSlots.slice(0, 6).map((slot) => (
                <Card
                  key={slot.id}
                  bg="gray.900"
                  border="1px solid"
                  borderColor="gray.700"
                  opacity={0.7}
                >
                  <CardBody>
                    <VStack spacing={3} align="start">
                      <HStack justify="space-between" w="full">
                        <Badge colorScheme="red">Taken</Badge>
                        <HStack>
                          <Text fontSize="xl">{robotNames[slot.robotType].emoji}</Text>
                          <Text fontSize="sm" color="gray.400">
                            {robotNames[slot.robotType].name}
                          </Text>
                        </HStack>
                      </HStack>
                      
                      <VStack align="start" spacing={1}>
                        <Text color="gray.300">
                          {new Date(slot.date).toLocaleDateString('en-US', { 
                            weekday: 'long', 
                            month: 'long', 
                            day: 'numeric' 
                          })}
                        </Text>
                        <Text color="gray.400">
                          {slot.startTime} - {slot.endTime}
                        </Text>
                      </VStack>

                      <Text fontSize="sm" color="gray.500">
                        Booked by {slot.bookedBy}
                      </Text>
                    </VStack>
                  </CardBody>
                </Card>
              ))}
            </SimpleGrid>
          </VStack>
        )}
      </VStack>
    </Container>
  );
};

export default BookingPage;