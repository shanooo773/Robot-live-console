import { extendTheme } from "@chakra-ui/react";

const theme = extendTheme({
  config: {
    initialColorMode: "light",
    useSystemColorMode: false,
  },
  colors: {
    // Modern Robotics Color System
    primary: {
      50: "#f8fafc",
      100: "#f1f5f9", 
      200: "#e2e8f0",
      300: "#cbd5e1",
      400: "#94a3b8",
      500: "#64748b",
      600: "#475569",
      700: "#334155",
      800: "#1e293b", // Primary Gray-800
      900: "#0f172a",
    },
    secondary: {
      50: "#fdf4ff",
      100: "#fae8ff",
      200: "#f3d4fe",
      300: "#e9a3ff",
      400: "#d946ef",
      500: "#c026d3",
      600: "#a21caf",
      700: "#86198f",
      800: "#701a75",
      900: "#581c87",
    },
    accent: {
      50: "#f0f9ff",
      100: "#e0f2fe",
      200: "#bae6fd",
      300: "#7dd3fc",
      400: "#38bdf8", // Bright accent
      500: "#0ea5e9",
      600: "#0284c7",
      700: "#0369a1",
      800: "#075985",
      900: "#0c4a6e",
    },
    brand: {
      50: "#e6f3ff",
      100: "#b3d9ff",
      200: "#80bfff",
      300: "#4da6ff",
      400: "#1a8cff",
      500: "#0066cc",
      600: "#0052a3",
      700: "#003d7a",
      800: "#002952",
      900: "#001429",
    },
    blue: {
      50: "#e6f3ff",
      100: "#b3d9ff",
      200: "#80bfff",
      300: "#4da6ff",
      400: "#1a8cff",
      500: "#0066cc",
      600: "#0052a3",
      700: "#003d7a",
      800: "#002952",
      900: "#001429",
    }
  },
  styles: {
    global: {
      body: {
        bg: "white",
        minHeight: "100vh",
        color: "gray.800",
        fontFamily: "system-ui, sans-serif",
      },
      "@keyframes pulse": {
        "0%, 100%": {
          opacity: 1,
        },
        "50%": {
          opacity: 0.8,
        },
      },
      "@keyframes float": {
        "0%, 100%": {
          transform: "translateY(0px)",
        },
        "50%": {
          transform: "translateY(-20px)",
        },
      },
      "@keyframes fadeInUp": {
        "0%": {
          opacity: 0,
          transform: "translateY(30px)",
        },
        "100%": {
          opacity: 1,
          transform: "translateY(0)",
        },
      },
    },
  },
  components: {
    Card: {
      baseStyle: {
        container: {
          bg: "white",
          border: "0",
          borderRadius: "12px",
          boxShadow: "0 1px 3px rgba(0, 0, 0, 0.1)",
          transition: "all 0.2s ease",
          _hover: {
            boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
            transform: "translateY(-2px)",
          },
        },
      },
    },
    Button: {
      baseStyle: {
        borderRadius: "8px",
        fontWeight: "600",
        transition: "all 0.2s ease",
      },
      variants: {
        solid: {
          bgGradient: "linear(to-r, cyan.400, blue.500)",
          color: "white",
          boxShadow: "0 8px 32px rgba(6, 182, 212, 0.3)",
          _hover: {
            bgGradient: "linear(to-r, cyan.300, blue.400)",
            boxShadow: "0 12px 40px rgba(6, 182, 212, 0.4)",
            transform: "translateY(-2px)",
          },
        },
        outline: {
          borderColor: "primary.800",
          color: "primary.800",
          _hover: {
            bg: "primary.50",
            borderColor: "primary.900",
          },
        },
      },
    },
    Heading: {
      baseStyle: {
        fontWeight: "700",
        lineHeight: "1.2",
      },
    },
    Text: {
      baseStyle: {
        lineHeight: "1.6",
      },
    },
  },
});

export default theme;
