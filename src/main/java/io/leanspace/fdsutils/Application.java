package io.leanspace.fdsutils;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.datatype.jsr310.JavaTimeModule;
import com.opencsv.CSVWriter;
import com.opencsv.ICSVWriter;
import io.leanspace.fdsutils.model.GroundStationWithName;
import io.leanspace.fdsutils.model.PropagationInputs;
import io.leanspace.flightdynamics.orekit.OrekitLibrary;
import io.leanspace.flightdynamics.orekit.model.GroundStation;
import io.leanspace.flightdynamics.orekit.model.GroundStationVisibility;
import io.leanspace.flightdynamics.orekit.model.OrbitData;
import io.leanspace.flightdynamics.orekit.model.OrbitElements;
import io.leanspace.flightdynamics.orekit.model.OrbitElements.CoordinateType;
import io.leanspace.flightdynamics.orekit.model.OrbitElements.Tle;
import io.leanspace.flightdynamics.orekit.model.OrbitPropagationParameters;
import io.leanspace.flightdynamics.orekit.model.OrbitPropagationResult;
import io.leanspace.flightdynamics.orekit.model.PropagationStep;
import io.leanspace.flightdynamics.orekit.model.PropagatorConfiguration;
import io.leanspace.flightdynamics.orekit.model.Spacecraft;
import io.leanspace.flightdynamics.orekit.orbit.propagation.NumericalOrbitPropagation;
import io.leanspace.flightdynamics.orekit.orbit.propagation.Sgp4OrbitPropagation;
import io.leanspace.flightdynamics.orekit.transforms.CoordinateTransformer;
import lombok.SneakyThrows;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.data.DataSource;
import org.orekit.frames.FramesFactory;
import org.orekit.gnss.navigation.RinexNavigation;
import org.orekit.gnss.navigation.RinexNavigationParser;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.analytical.gnss.data.GNSSConstants;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.Instant;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.UUID;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Application {

  public static void main(String[] args) {
    OrekitLibrary.setup();
    runTests();
  }

  private static void runTests() {
    propagateTle();
    //parseRinexData("src/main/resources/rinex/ABPO00MDG_R_20233602200_01H_CN.rnx");
    //parseRinexData("src/main/resources/rinex/MET300FIN_R_20240581800_01H_MN.rnx");
    //generateGroundStationVisibilities();
  }

  private static PropagationInputs getPropagationInputs() {

    final int secondsInDay = 86400;
    final int propagationDuration = 30 * secondsInDay;
    final int propagationStepInSeconds = 60;
    final String tleLine1 = "1 42829U 17042E   19338.93232017 +.00000015 +00000-0 +68468-5 0  9993";
    final String tleLine2 = "2 42829 097.5429 224.5636 0013299 211.4408 148.6018 14.91030680130155";
    final EnumSet<CoordinateType> coordinateTypesToUse = EnumSet.of(CoordinateType.CARTESIAN_ECI);
    return PropagationInputs.builder()
      .tle(new Tle(tleLine1, tleLine2))
      .startTime(Instant.now())
      .stopTime(Instant.now().plusSeconds(propagationDuration))
      .step(propagationStepInSeconds)
      .coordinateTypesToUse(coordinateTypesToUse)
      .build();
  }

  private static void propagateTle() {
    OrbitElements.Tle tle = OrbitElements.Tle.builder()
      .line1("1 25544U 98067A   24142.35003124  .00022843  00000-0  38371-3 0  9995")
      .line2("2 25544  51.6390  88.3709 0003333 191.4959 306.2513 15.51667899454382")
      .build();
    Sgp4OrbitPropagation sgp4 = new Sgp4OrbitPropagation(tle);
    int stepInSeconds = 60;
    sgp4.setStepHandler(stepInSeconds, EnumSet.of(CoordinateType.GEODETIC, CoordinateType.CARTESIAN_ECEF));
    sgp4.propagate(Instant.parse("2024-05-21T08:24:02.699Z"), Instant.parse("2024-05-21T08:24:02.699Z").plusSeconds(60L * 4));

    final List<PropagationStep> results = sgp4.getPropagationResults();
    try (CSVWriter writer = new CSVWriter(new FileWriter("orbit_propagation.csv"), ICSVWriter.DEFAULT_SEPARATOR,
      ICSVWriter.NO_QUOTE_CHARACTER,
      ICSVWriter.DEFAULT_ESCAPE_CHARACTER,
      ICSVWriter.DEFAULT_LINE_END)) {
      writer.writeNext(new String[]{"Timestamp", "Latitude (rad)", "Longitude (rad)", "Altitude (m)", "Ground Speed (m/s)"});
      for (PropagationStep result : results) {
        OrbitElements.CartesianElements ecef = result.getCartesianElementsEcef();
        double groundSpeed = Math.sqrt(ecef.getVelocityInMetersPerSecond().getX() * ecef.getVelocityInMetersPerSecond().getX() + ecef.getVelocityInMetersPerSecond().getY() * ecef.getVelocityInMetersPerSecond().getY() + ecef.getVelocityInMetersPerSecond().getZ() * ecef.getVelocityInMetersPerSecond().getZ());
        String[] data = {
          result.getTimestamp().toString(),
          String.valueOf(Math.toRadians(result.getGeodeticElements().getLatitudeInDegrees())),
          String.valueOf(Math.toRadians(result.getGeodeticElements().getLongitudeInDegrees())),
          String.valueOf(result.getGeodeticElements().getAltitudeInMeters()),
          String.valueOf(groundSpeed)
        };
        writer.writeNext(data);
      }
    } catch (IOException ignored) {}
  }

  private static void generateOemFile() {

    final PropagationInputs propagationInputs = getPropagationInputs();

    final OrbitPropagationParameters inputParameters = OrbitPropagationParameters.builder()
      .spacecraft(Spacecraft.builder()
        .name("TECHNOSAT")
        .id("42829")
        .build())
      .orbitData(OrbitData.builder()
        .date(propagationInputs.getStartTime())
        .orbitElements(OrbitElements.builder()
          .type(OrbitElements.CoordinateType.TLE)
          .tle(OrbitElements.Tle.builder()
            .line1(propagationInputs.getTle().getLine1())
            .line2(propagationInputs.getTle().getLine2())
            .build())
          .build())
        .build())
      .propagatorConfiguration(PropagatorConfiguration.builder()
        .isAnalytical(true)
        .build())
      .startDate(propagationInputs.getStartTime())
      .step(propagationInputs.getStep())
      .duration(propagationInputs.getStopTime().getEpochSecond() - propagationInputs.getStartTime().getEpochSecond())
      .generateOem(true)
      .build();

    final OrbitPropagationResult output = new NumericalOrbitPropagation()
        .run(inputParameters, propagationInputs.getCoordinateTypesToUse());

    try (FileWriter writer = new FileWriter("output.oem")) {
      writer.write(output.getOemFile());
    } catch (IOException e) {
      System.err.println("Error writing to the file: " + e.getMessage());
    }
  }

  @SneakyThrows
  private static List<GroundStationWithName> getGsList() {
    final String filePath = "src/main/resources/groundStations.json";
    String jsonList = new String(Files.readAllBytes(Paths.get(filePath)));

    ObjectMapper objectMapper = new ObjectMapper();
    List<Map<String, Object>> locations = objectMapper.readValue(jsonList, new TypeReference<>() {});
    return locations.stream()
      .map(location -> GroundStationWithName.builder()
        .groundStation(GroundStation.builder()
          .id(UUID.randomUUID())
          .latitude((Double) location.get("latitude"))
          .longitude((Double) location.get("longitude"))
          .elevation(((Integer) location.get("elevation")).doubleValue())
          .build())
        .name(location.get("name").toString())
        .build())
      .collect(Collectors.toList());
  }

  @SneakyThrows
  private static List<Tle> getTleList() {
    final String filePath = "src/main/resources/tles.txt";
    final List<String> lines = Files.lines(Paths.get(filePath)).collect(Collectors.toList());
    return Stream.iterate(0, i -> i + 2)
      .limit(lines.size() / 2)
      .map(i -> new Tle(lines.get(i), (i + 1 < lines.size()) ? lines.get(i + 1) : null))
      .collect(Collectors.toList());
  }

  @SneakyThrows
  public static void generateGroundStationVisibilities() {
    List<GroundStationWithName> groundStationsWithNames = getGsList();
    final List<Tle> tles = getTleList();

    final ObjectMapper objectMapper = new ObjectMapper();
    objectMapper.registerModule(new JavaTimeModule());
    objectMapper.configure(SerializationFeature.WRITE_DATES_AS_TIMESTAMPS, false);
    objectMapper.configure(SerializationFeature.WRITE_DATE_TIMESTAMPS_AS_NANOSECONDS, false);

    List<GroundStation> groundStationsList = groundStationsWithNames.stream()
      .map(GroundStationWithName::getGroundStation)
      .collect(Collectors.toList());

    int i = 0;
    for (Tle tle : tles) {
      final PropagationInputs inputs = PropagationInputs.builder()
        .tle(tle)
        .startTime(Instant.now())
        .stopTime(Instant.now().plusSeconds(3 * 60 * 60))
        .step(60)
        .groundStationList(groundStationsList)
        .build();
      final Sgp4OrbitPropagation sgp4 = new Sgp4OrbitPropagation(inputs.getTle());

      sgp4.setStepHandler(inputs.getStep());

      sgp4.addVisibilityDetector(inputs.getStartTime(), inputs.getStopTime(), inputs.getGroundStationList());
      sgp4.propagate(inputs.getStartTime(), inputs.getStopTime());

      final List<GroundStationVisibility> visibilities = sgp4.getVisibilities();

      // Create a list in the required format
      List<Map<String, Object>> jsonList = visibilities.stream()
        .map(visibility -> {
          Map<String, Object> jsonMap = Map.of(
            "start", visibility.getAos(),
            "end", visibility.getLos(),
            "groundstationName", Objects.requireNonNull(groundStationsWithNames.stream()
              .filter(item -> item.getGroundStation().getId().equals(visibility.getGroundStationId()))
              .findFirst()
              .map(GroundStationWithName::getName)
              .orElse(null))
          );
          return jsonMap;
        })
        .collect(Collectors.toList());

      char letter = (char) ('A' + i / 2);
      Files.write(Path.of(String.valueOf(letter) + (i%2 + 1) + " Passes.json"), objectMapper.writerWithDefaultPrettyPrinter().writeValueAsString(jsonList).getBytes());
      i++;
    }
  }

  @SneakyThrows
  public static void parseRinexData(String filePath) {

    RinexNavigationParser parser = new RinexNavigationParser();
    RinexNavigation navigation = parser.parse(new DataSource(filePath));

    navigation.getGPSNavigationMessages().forEach((sat, message) -> {
      System.out.println("GPS - Satellite " + sat);
      message.forEach((o) -> {
        AbsoluteDate epoch = o.getEpochToc();
        Double sma = o.getSma();
        Double ecc = o.getE();
        Double inc = o.getI0();
        Double ran = o.getOmega0();
        Double aop = o.getPa();
        Double man = o.getM0();

        KeplerianOrbit keplerianOrbit = new KeplerianOrbit(
          sma,
          ecc,
          inc,
          aop,
          ran,
          man,
          PositionAngle.MEAN,
          FramesFactory.getGCRF(),
          epoch,
          Constants.WGS84_EARTH_MU
        );
        PVCoordinates pvCoordinates = keplerianOrbit.getPVCoordinates();
        System.out.println("Cartesian vector: " + pvCoordinates.toString());
      });
    });

    navigation.getGlonassNavigationMessages().forEach((sat, message) -> {
      System.out.println("GLONASS - Satellite " + sat);
      message.forEach((o) -> {

        AbsoluteDate epoch = o.getEpochToc();
        Double px = o.getX();
        Double py = o.getY();
        Double pz = o.getZ();
        Double vx = o.getXDot();
        Double vy = o.getYDot();
        Double vz = o.getZDot();

        CartesianOrbit cartesianOrbit = new CartesianOrbit(
          new TimeStampedPVCoordinates(epoch, new Vector3D(px, py, pz), new Vector3D(vx, vy, vz)),
          FramesFactory.getEME2000(),
          Constants.WGS84_EARTH_MU
        );
        System.out.println("Cartesian vector: " + cartesianOrbit.getPVCoordinates().toString());
      });
    });

    navigation.getGalileoNavigationMessages().forEach((sat, message) -> {
      System.out.println("Galileo - Satellite " + sat);
      message.forEach((o) -> {
        AbsoluteDate epoch = o.getEpochToc();
        Double sma = o.getSma();
        Double ecc = o.getE();
        Double inc = o.getI0();
        Double ran = o.getOmega0();
        Double aop = o.getPa();
        Double man = o.getM0();

        KeplerianOrbit keplerianOrbit = new KeplerianOrbit(
          sma,
          ecc,
          inc,
          aop,
          ran,
          man,
          PositionAngle.MEAN,
          FramesFactory.getGCRF(),
          epoch,
          Constants.WGS84_EARTH_MU
        );
        PVCoordinates pvCoordinates = keplerianOrbit.getPVCoordinates();
        System.out.println("Cartesian vector: " + pvCoordinates.toString());
      });
    });

    navigation.getBeidouNavigationMessages().forEach((sat, message) -> {
      System.out.println("Beidou - Satellite " + sat);
      message.forEach((o) -> {
        AbsoluteDate epoch = o.getEpochToc();
        Double sma = o.getSma();
        Double ecc = o.getE();
        Double inc = o.getI0();
        Double ran = o.getOmega0();
        Double aop = o.getPa();
        Double man = o.getM0();

        KeplerianOrbit keplerianOrbit = new KeplerianOrbit(
          sma,
          ecc,
          inc,
          aop,
          ran,
          man,
          PositionAngle.MEAN,
          FramesFactory.getGCRF(),
          epoch,
          Constants.WGS84_EARTH_MU
        );
        PVCoordinates pvCoordinates = keplerianOrbit.getPVCoordinates();
        System.out.println("Cartesian vector: " + pvCoordinates.toString());
      });
    });

    navigation.getIRNSSNavigationMessages().forEach((sat, message) -> {
      System.out.println("IRNSS - Satellite " + sat);
      message.forEach((o) -> {
        AbsoluteDate epoch = o.getEpochToc();
        Double sma = o.getSma();
        Double ecc = o.getE();
        Double inc = o.getI0();
        Double ran = o.getOmega0();
        Double aop = o.getPa();
        Double man = o.getM0();

        KeplerianOrbit keplerianOrbit = new KeplerianOrbit(
          sma,
          ecc,
          inc,
          aop,
          ran,
          man,
          PositionAngle.MEAN,
          FramesFactory.getGCRF(),
          epoch,
          Constants.WGS84_EARTH_MU
        );
        PVCoordinates pvCoordinates = keplerianOrbit.getPVCoordinates();
        System.out.println("Cartesian vector: " + pvCoordinates.toString());
      });
    });

    navigation.getQZSSNavigationMessages().forEach((sat, message) -> {
      System.out.println("QZSS - Satellite " + sat);
      message.forEach((o) -> {
        AbsoluteDate epoch = o.getEpochToc();
        Double sma = o.getSma();
        Double ecc = o.getE();
        Double inc = o.getI0();
        Double ran = o.getOmega0();
        Double aop = o.getPa();
        Double man = o.getM0();

        KeplerianOrbit keplerianOrbit = new KeplerianOrbit(
          sma,
          ecc,
          inc,
          aop,
          ran,
          man,
          PositionAngle.MEAN,
          FramesFactory.getGCRF(),
          epoch,
          Constants.WGS84_EARTH_MU
        );
        PVCoordinates pvCoordinates = keplerianOrbit.getPVCoordinates();
        System.out.println("Cartesian vector: " + pvCoordinates.toString());
      });
    });

    navigation.getSBASNavigationMessages().forEach((sat, message) -> {
      System.out.println("SBAS - Satellite " + sat);
      message.forEach((o) -> {

        AbsoluteDate epoch = o.getEpochToc();
        Double px = o.getX();
        Double py = o.getY();
        Double pz = o.getZ();
        Double vx = o.getXDot();
        Double vy = o.getYDot();
        Double vz = o.getZDot();

        CoordinateTransformer coordinateTransformer = new CoordinateTransformer();
        TimeStampedPVCoordinates eciCoordinates = coordinateTransformer.toEci(FramesFactory.getITRF(IERSConventions.IERS_2010, true),
          new TimeStampedPVCoordinates(epoch, new Vector3D(px, py, pz), new Vector3D(vx, vy, vz)));

        CartesianOrbit cartesianOrbit = new CartesianOrbit(
          eciCoordinates,
          FramesFactory.getGCRF(),
          GNSSConstants.SBAS_MU
        );
        System.out.println("Cartesian vector: " + cartesianOrbit.getPVCoordinates().toString());
      });
    });
  }

}
