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
  Table,
  Thead,
  Tbody,
  Tr,
  Th,
  Td,
  TableContainer,
  Stat,
  StatLabel,
  StatNumber,
  StatHelpText,
  Menu,
  MenuButton,
  MenuList,
  MenuItem,
  AlertDialog,
  AlertDialogBody,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogContent,
  AlertDialogOverlay,
  useDisclosure,
} from "@chakra-ui/react";
import { useState, useEffect, useRef } from "react";
import { getAllUsers, getAllBookings, getAdminStats, updateBookingStatus, deleteBooking } from "../api";
import { ChevronDownIcon, DeleteIcon, EditIcon } from "@chakra-ui/icons";

const AdminDashboard = ({ user, authToken, onBack, onLogout }) => {
  const [stats, setStats] = useState(null);
  const [users, setUsers] = useState([]);
  const [bookings, setBookings] = useState([]);
  const [isLoading, setIsLoading] = useState(false);
  const [selectedBooking, setSelectedBooking] = useState(null);
  const toast = useToast();
  
  const { isOpen, onOpen, onClose } = useDisclosure();
  const cancelRef = useRef();

  useEffect(() => {
    loadDashboardData();
  }, [authToken]);

  const loadDashboardData = async () => {
    setIsLoading(true);
    try {
      const [statsData, usersData, bookingsData] = await Promise.all([
        getAdminStats(authToken),
        getAllUsers(authToken),
        getAllBookings(authToken)
      ]);
      
      setStats(statsData);
      setUsers(usersData);
      setBookings(bookingsData);
    } catch (error) {
      console.error('Error loading dashboard data:', error);
      toast({
        title: "Error loading dashboard",
        description: "Failed to load admin dashboard data",
        status: "error",
        duration: 5000,
        isClosable: true,
      });
    } finally {
      setIsLoading(false);
    }
  };

  const handleUpdateBookingStatus = async (bookingId, newStatus) => {
    try {
      await updateBookingStatus(bookingId, newStatus, authToken);
      await loadDashboardData();
      toast({
        title: "Booking updated",
        description: `Booking status changed to ${newStatus}`,
        status: "success",
        duration: 3000,
        isClosable: true,
      });
    } catch (error) {
      console.error('Error updating booking:', error);
      toast({
        title: "Update failed",
        description: "Failed to update booking status",
        status: "error",
        duration: 3000,
        isClosable: true,
      });
    }
  };

  const handleDeleteBooking = async () => {
    try {
      await deleteBooking(selectedBooking.id, authToken);
      await loadDashboardData();
      onClose();
      toast({
        title: "Booking deleted",
        description: "Booking has been successfully deleted",
        status: "success",
        duration: 3000,
        isClosable: true,
      });
    } catch (error) {
      console.error('Error deleting booking:', error);
      toast({
        title: "Delete failed",
        description: "Failed to delete booking",
        status: "error",
        duration: 3000,
        isClosable: true,
      });
    }
  };

  const confirmDeleteBooking = (booking) => {
    setSelectedBooking(booking);
    onOpen();
  };

  const formatDate = (dateString) => {
    return new Date(dateString).toLocaleDateString('en-US', {
      year: 'numeric',
      month: 'short',
      day: 'numeric'
    });
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'active':
        return 'green';
      case 'cancelled':
        return 'red';
      case 'completed':
        return 'blue';
      default:
        return 'gray';
    }
  };

  return (
    <Container maxW="7xl" py={8}>
      <VStack spacing={8}>
        {/* Header */}
        <HStack w="full" justify="space-between">
          <VStack align="start" spacing={1}>
            <Text fontSize="3xl" fontWeight="bold" color="white">
              Admin Dashboard
            </Text>
            <HStack>
              <Avatar size="sm" name={user.name} />
              <Text color="gray.300">Welcome, {user.name}</Text>
              <Badge colorScheme="purple">Admin</Badge>
            </HStack>
          </VStack>
          <HStack spacing={3}>
            <Button variant="ghost" onClick={onBack} color="gray.400">
              ← Back to Booking
            </Button>
            <Button variant="ghost" onClick={onLogout} color="gray.400">
              Logout
            </Button>
          </HStack>
        </HStack>

        {/* Statistics Cards */}
        {stats && (
          <SimpleGrid columns={{ base: 1, md: 2, lg: 4 }} spacing={6} w="full">
            <Card bg="blue.900" border="1px solid" borderColor="blue.600">
              <CardBody>
                <Stat>
                  <StatLabel color="blue.100">Total Users</StatLabel>
                  <StatNumber color="white" fontSize="3xl">{stats.total_users}</StatNumber>
                  <StatHelpText color="blue.200">Registered users</StatHelpText>
                </Stat>
              </CardBody>
            </Card>

            <Card bg="green.900" border="1px solid" borderColor="green.600">
              <CardBody>
                <Stat>
                  <StatLabel color="green.100">Total Bookings</StatLabel>
                  <StatNumber color="white" fontSize="3xl">{stats.total_bookings}</StatNumber>
                  <StatHelpText color="green.200">All time bookings</StatHelpText>
                </Stat>
              </CardBody>
            </Card>

            <Card bg="purple.900" border="1px solid" borderColor="purple.600">
              <CardBody>
                <Stat>
                  <StatLabel color="purple.100">Active Bookings</StatLabel>
                  <StatNumber color="white" fontSize="3xl">{stats.active_bookings}</StatNumber>
                  <StatHelpText color="purple.200">Currently active</StatHelpText>
                </Stat>
              </CardBody>
            </Card>

            <Card bg="orange.900" border="1px solid" borderColor="orange.600">
              <CardBody>
                <Stat>
                  <StatLabel color="orange.100">Utilization</StatLabel>
                  <StatNumber color="white" fontSize="3xl">
                    {stats.total_bookings > 0 ? Math.round((stats.active_bookings / stats.total_bookings) * 100) : 0}%
                  </StatNumber>
                  <StatHelpText color="orange.200">Active rate</StatHelpText>
                </Stat>
              </CardBody>
            </Card>
          </SimpleGrid>
        )}

        {/* Recent Users */}
        <Card w="full" bg="gray.800" border="1px solid" borderColor="gray.600">
          <CardHeader>
            <Text fontSize="lg" fontWeight="bold" color="white">
              Recent Users
            </Text>
          </CardHeader>
          <CardBody>
            <TableContainer>
              <Table variant="simple" size="sm">
                <Thead>
                  <Tr>
                    <Th color="gray.300">Name</Th>
                    <Th color="gray.300">Email</Th>
                    <Th color="gray.300">Role</Th>
                    <Th color="gray.300">Joined</Th>
                  </Tr>
                </Thead>
                <Tbody>
                  {users.slice(0, 5).map((userData) => (
                    <Tr key={userData.id}>
                      <Td color="white">
                        <HStack>
                          <Avatar size="xs" name={userData.name} />
                          <Text>{userData.name}</Text>
                        </HStack>
                      </Td>
                      <Td color="gray.300">{userData.email}</Td>
                      <Td>
                        <Badge colorScheme={userData.role === 'admin' ? 'purple' : 'blue'}>
                          {userData.role}
                        </Badge>
                      </Td>
                      <Td color="gray.400">{formatDate(userData.created_at)}</Td>
                    </Tr>
                  ))}
                </Tbody>
              </Table>
            </TableContainer>
          </CardBody>
        </Card>

        {/* All Bookings */}
        <Card w="full" bg="gray.800" border="1px solid" borderColor="gray.600">
          <CardHeader>
            <Text fontSize="lg" fontWeight="bold" color="white">
              All Bookings
            </Text>
          </CardHeader>
          <CardBody>
            <TableContainer>
              <Table variant="simple" size="sm">
                <Thead>
                  <Tr>
                    <Th color="gray.300">User</Th>
                    <Th color="gray.300">Robot Type</Th>
                    <Th color="gray.300">Date</Th>
                    <Th color="gray.300">Time</Th>
                    <Th color="gray.300">Status</Th>
                    <Th color="gray.300">Actions</Th>
                  </Tr>
                </Thead>
                <Tbody>
                  {bookings.map((booking) => (
                    <Tr key={booking.id}>
                      <Td color="white">
                        <VStack align="start" spacing={0}>
                          <Text fontWeight="semibold">{booking.user_name}</Text>
                          <Text fontSize="xs" color="gray.400">{booking.user_email}</Text>
                        </VStack>
                      </Td>
                      <Td color="gray.300">
                        <Badge colorScheme="cyan">{booking.robot_type}</Badge>
                      </Td>
                      <Td color="gray.300">{formatDate(booking.date)}</Td>
                      <Td color="gray.300">{booking.start_time} - {booking.end_time}</Td>
                      <Td>
                        <Badge colorScheme={getStatusColor(booking.status)}>
                          {booking.status}
                        </Badge>
                      </Td>
                      <Td>
                        <HStack spacing={2}>
                          <Menu>
                            <MenuButton as={Button} size="xs" rightIcon={<ChevronDownIcon />}>
                              Status
                            </MenuButton>
                            <MenuList>
                              <MenuItem onClick={() => handleUpdateBookingStatus(booking.id, 'active')}>
                                Mark Active
                              </MenuItem>
                              <MenuItem onClick={() => handleUpdateBookingStatus(booking.id, 'completed')}>
                                Mark Completed
                              </MenuItem>
                              <MenuItem onClick={() => handleUpdateBookingStatus(booking.id, 'cancelled')}>
                                Mark Cancelled
                              </MenuItem>
                            </MenuList>
                          </Menu>
                          <Button
                            size="xs"
                            colorScheme="red"
                            onClick={() => confirmDeleteBooking(booking)}
                          >
                            Delete
                          </Button>
                        </HStack>
                      </Td>
                    </Tr>
                  ))}
                </Tbody>
              </Table>
            </TableContainer>
          </CardBody>
        </Card>
      </VStack>

      {/* Delete Confirmation Dialog */}
      <AlertDialog
        isOpen={isOpen}
        leastDestructiveRef={cancelRef}
        onClose={onClose}
      >
        <AlertDialogOverlay>
          <AlertDialogContent bg="gray.800" border="1px solid" borderColor="gray.600">
            <AlertDialogHeader fontSize="lg" fontWeight="bold" color="white">
              Delete Booking
            </AlertDialogHeader>

            <AlertDialogBody color="gray.300">
              Are you sure you want to delete this booking? This action cannot be undone.
              {selectedBooking && (
                <Box mt={4} p={4} bg="gray.700" borderRadius="md">
                  <Text><strong>User:</strong> {selectedBooking.user_name}</Text>
                  <Text><strong>Robot:</strong> {selectedBooking.robot_type}</Text>
                  <Text><strong>Date:</strong> {selectedBooking.date}</Text>
                  <Text><strong>Time:</strong> {selectedBooking.start_time} - {selectedBooking.end_time}</Text>
                </Box>
              )}
            </AlertDialogBody>

            <AlertDialogFooter>
              <Button ref={cancelRef} onClick={onClose}>
                Cancel
              </Button>
              <Button colorScheme="red" onClick={handleDeleteBooking} ml={3}>
                Delete
              </Button>
            </AlertDialogFooter>
          </AlertDialogContent>
        </AlertDialogOverlay>
      </AlertDialog>
    </Container>
  );
};

export default AdminDashboard;