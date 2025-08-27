import { extendTheme } from "@chakra-ui/react";

const theme = extendTheme({
  config: {
    initialColorMode: "dark",
    useSystemColorMode: false,
  },
  colors: {
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
    },
    // Electric blue accents for modern design
    electric: {
      50: "#e6f9ff",
      100: "#b3ecff",
      200: "#80dfff",
      300: "#4dd2ff",
      400: "#1ac5ff",
      500: "#00bfff", // Primary electric blue
      600: "#0099cc",
      700: "#007399",
      800: "#004d66",
      900: "#002633",
    },
    // Deep navy/black palette
    navy: {
      50: "#f0f4f8",
      100: "#d9e2ec",
      200: "#bcccdc",
      300: "#9fb3c8",
      400: "#829ab1",
      500: "#627d98",
      600: "#486581",
      700: "#334e68",
      800: "#243b53",
      900: "#102a43",
      950: "#0a1929", // Deepest navy/black
    }
  },
  styles: {
    global: {
      body: {
        // Deeper navy/black gradient background
        bg: "linear-gradient(135deg, #0a1929 0%, #102a43 25%, #243b53 50%, #334e68 75%, #486581 100%)",
        minHeight: "100vh",
      },
      // Enhanced animations for modern feel
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
      "@keyframes shimmer": {
        "0%": {
          backgroundPosition: "-200% 0",
        },
        "100%": {
          backgroundPosition: "200% 0",
        },
      },
      "@keyframes slideInUp": {
        "0%": {
          transform: "translateY(30px)",
          opacity: 0,
        },
        "100%": {
          transform: "translateY(0)",
          opacity: 1,
        },
      },
      "@keyframes fadeInScale": {
        "0%": {
          transform: "scale(0.95)",
          opacity: 0,
        },
        "100%": {
          transform: "scale(1)",
          opacity: 1,
        },
      },
    },
  },
  components: {
    Card: {
      baseStyle: {
        container: {
          // Pure white cards with modern glassmorphism
          bg: "rgba(255, 255, 255, 0.95)",
          backdropFilter: "blur(20px)",
          border: "1px solid",
          borderColor: "rgba(255, 255, 255, 0.2)",
          borderRadius: "20px",
          boxShadow: "0 8px 32px rgba(0, 0, 0, 0.1), 0 4px 16px rgba(0, 0, 0, 0.05)",
          transition: "all 0.4s cubic-bezier(0.4, 0, 0.2, 1)",
          _hover: {
            transform: "translateY(-8px)",
            boxShadow: "0 20px 40px rgba(0, 0, 0, 0.15), 0 8px 24px rgba(0, 0, 0, 0.1)",
            borderColor: "rgba(0, 191, 255, 0.3)",
          },
        },
      },
      variants: {
        // Glass card variant for special sections
        glass: {
          container: {
            bg: "rgba(255, 255, 255, 0.1)",
            backdropFilter: "blur(20px)",
            border: "1px solid",
            borderColor: "rgba(255, 255, 255, 0.2)",
            borderRadius: "24px",
            boxShadow: "0 8px 32px rgba(255, 255, 255, 0.1)",
            transition: "all 0.4s cubic-bezier(0.4, 0, 0.2, 1)",
            _hover: {
              transform: "translateY(-6px)",
              boxShadow: "0 20px 40px rgba(255, 255, 255, 0.15)",
              borderColor: "rgba(0, 191, 255, 0.4)",
            },
          },
        },
      },
    },
    Button: {
      baseStyle: {
        borderRadius: "full",
        fontWeight: "bold",
        transition: "all 0.3s cubic-bezier(0.4, 0, 0.2, 1)",
        _focus: {
          boxShadow: "0 0 0 3px rgba(0, 191, 255, 0.3)",
        },
      },
      variants: {
        solid: {
          // Electric blue gradient for CTAs
          bg: "linear-gradient(135deg, #00bfff 0%, #1ac5ff 50%, #0099cc 100%)",
          color: "white",
          boxShadow: "0 4px 20px rgba(0, 191, 255, 0.4)",
          _hover: {
            bg: "linear-gradient(135deg, #1ac5ff 0%, #4dd2ff 50%, #00bfff 100%)",
            boxShadow: "0 8px 30px rgba(0, 191, 255, 0.6)",
            transform: "translateY(-3px)",
          },
          _active: {
            transform: "translateY(-1px)",
            boxShadow: "0 4px 15px rgba(0, 191, 255, 0.5)",
          },
        },
        outline: {
          borderColor: "electric.500",
          color: "electric.500",
          borderWidth: "2px",
          _hover: {
            bg: "rgba(0, 191, 255, 0.1)",
            borderColor: "electric.400",
            color: "electric.400",
            transform: "translateY(-2px)",
            boxShadow: "0 6px 20px rgba(0, 191, 255, 0.3)",
          },
        },
        ghost: {
          color: "electric.500",
          _hover: {
            bg: "rgba(0, 191, 255, 0.1)",
            color: "electric.400",
            transform: "translateY(-2px)",
          },
        },
      },
      sizes: {
        lg: {
          px: 8,
          py: 6,
          fontSize: "lg",
          minH: "56px",
        },
      },
    },
    Text: {
      baseStyle: {
        lineHeight: "1.6",
      },
      variants: {
        gradient: {
          bgGradient: "linear(135deg, electric.400, electric.600)",
          bgClip: "text",
          fontWeight: "bold",
        },
      },
    },
  },
});

export default theme;
